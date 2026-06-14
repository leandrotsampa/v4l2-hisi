# hi_v4l2 — Driver V4L2 mem2mem de vídeo (Hisilicon Hi3798Cv200)

Módulo `hi_v4l2.ko` que expõe **decoder** e **encoder** V4L2 *stateful*
(`V4L2_CAP_VIDEO_M2M`) sobre a camada de kernel `HI_DRV_VDEC_*` / `HI_DRV_VENC_*`
(vdec_v2.0 / venc_v2.0) — a mesma usada pelo AVPLAY, comprovada em produção.
Permite que apps genéricas (GStreamer, FFmpeg, tvheadend) usem o HW de vídeo sem
as libs proprietárias. O caminho OMX (instável) foi evitado.

## Integração no kernel (padrão dvb-hisi)

Este driver mora em `drivers/msp/drv/v4l2-hisi/` (git próprio, clonado aqui), no
**mesmo padrão do `dvb-hisi`**:

- O `Makefile` inclui `base.mak`, declara `MOD_NAME := hi_v4l2` + a lista de
  objetos, e tem as regras `all/clean/install`.
- O **parent** `drivers/msp/drv/Makefile` o adiciona automaticamente quando o dir
  existe: `ifneq (,$(shell ls -d .../v4l2-hisi)) objects += v4l2-hisi endif` →
  `obj-y += v4l2-hisi/` desce no diretório durante o build do kernel.
- **Diferença vs dvb-hisi:** `dvb-hisi` é *built-in* (`obj-$(HI_DRV_BUILDTYPE)`,
  =y com `CFG_HI_MSP_BUILDIN=y`). O `hi_v4l2` é **sempre módulo (`obj-m`)** porque
  depende do framework V4L2 mem2mem + videobuf2 (`v4l2-mem2mem.ko`,
  `videobuf2-*.ko`), que são `=m` neste defconfig — built-in não linkaria
  (símbolos `v4l2_m2m_*`/`vb2_*` não resolvidos). Os símbolos `HI_DRV_VDEC_*`/
  `HI_DRV_VENC_*` vêm do vmlinux (msp built-in), resolvidos pelo `Module.symvers`.

## Arquivos

| Arquivo | Papel |
|---|---|
| `hi_v4l2_core.c` | `module_init/exit`; registra decoder + encoder |
| `hi_v4l2_dec.c` | Decoder m2m (coded → NV12/NV21) + **de-tile SW** + capture kthread |
| `hi_v4l2_enc.c` | Encoder m2m (NV12/NV21 → H264) + capture kthread + controles |
| `hi_vdec_hal.c/.h` | Wrapper sobre `HI_DRV_VDEC_*` |
| `hi_venc_hal.c/.h` | Wrapper sobre `HI_DRV_VENC_*` |
| `hi_v4l2_fmt.c/.h` | Tabela `V4L2_PIX_FMT_* ↔ HI_UNF_VCODEC_TYPE_E` |
| `hi_v4l2_drv.h` | Hooks de registro dec/enc |
| `hi_v4l2_zc_test.c` | Sonda zero-copy isolada (gated `HI_V4L2_ZC_SELFTEST`; experimental) |
| `test/` | Harnesses userspace (decode/encode m2m, ffmpeg, nv12cmp, ubootenv) |

## Dispositivos / formatos

- **Decoder** `/dev/video0` (m2m):
  - OUTPUT (coded): `MPEG2, MPEG4/XVID, H264, HEVC, VC1, VP8, VP9, MJPEG`
  - CAPTURE: `NV12`/`NV21` (vb2-vmalloc, **cached**)
  - de-tile SW **default ON** (param `detile`); `EOS`/`SOURCE_CHANGE`, `DECODER_CMD`
- **Encoder** `/dev/video1` (m2m):
  - OUTPUT: `NV12`/`NV21` (vb2-dma-contig → phys p/ o VEDU)
  - CAPTURE: **`H264`** (único; o VEDU do Hi3798Cv200 não tem HEVC/JPEG em HW)
  - Controles: `BITRATE`, `GOP_SIZE`, `H264_PROFILE`, `H264_LEVEL`

## Build

Como parte do kernel (o parent Makefile já o inclui):
```sh
# na raiz da SDK, com o ambiente configurado (cfg.mak + source env.sh)
make linux              # builda o kernel; hi_v4l2.ko sai como módulo
```
Ou só o módulo, contra este kernel:
```sh
make -C <hisilicon-kernel> M=drivers/msp/drv/v4l2-hisi modules \
     ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
```
Os módulos base do framework (`=m`) precisam existir:
```sh
make -C <hisilicon-kernel> drivers/media/v4l2-core/v4l2-mem2mem.ko \
                           drivers/media/v4l2-core/videobuf2-dma-contig.ko
```

## Deploy / carga

```sh
insmod videobuf2-dma-contig.ko
insmod v4l2-mem2mem.ko
insmod hi_v4l2.ko            # detile=1 é o default; cria /dev/video0 (dec) + video1 (enc)
```
(videodev / videobuf2-core / -vmalloc são built-in.)

## Estado (validado em HW, 2026-06-14)

- **Decoder**: 6 codecs **pixel-validados** (H264/MPEG2/VP8/VP9/HEVC/MPEG-4),
  de-tile correto, `v4l2-compliance 45/47`. **~37 fps@1080p / 75 fps@720p**.
- **Encoder**: H264 **real-time** (~42-52 fps@1080p / ~91 fps@720p), saída válida
  (round-trip), EOS limpo. Dirigível por gst `v4l2h264enc` / ffmpeg `h264_v4l2m2m`.
- Ambos dirigíveis por gst (`v4l2{h264,vp8,vp9}dec`, `v4l2h264enc`).
- Modelo **desacoplado** (capture kthread + back-pressure) em dec e enc.

## Limitações (HW/arquitetura — não são bugs)

- Encoder **só H264** (VEDU não tem HEVC/JPEG).
- **1080p60 não em tempo-real** (decode ~38 fps limitado pela de-tile SW ~19 ms/frame;
  encode ~52 fps); **720p60 ok**.
- Decode H264 capado ~1080p (VDH level 4.1).
- **Display de HD na tela não é via V4L2-GLES** (Mali fbdev: `glTexSubImage2D` é
  upload-bound, sem `dma_buf_import`); HD liso na tela = overlay VO do vendor.
- Params `zc`/`win`/`vp` e `hi_v4l2_zc_test.c` são experimentais (default OFF).
