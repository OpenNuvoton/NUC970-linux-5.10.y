/*
 * Copyright (c) 2023 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/hardware.h>

#include "nuc970-audio.h"

static const struct snd_pcm_hardware nuc970_pcm_hardware = {
	.info           = SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME,
	.formats        = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
	.rates          = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000,
	.channels_min       = 1,
	.channels_max       = 2,
	.buffer_bytes_max   = 4*1024,
	.period_bytes_min   = 1*1024,
	.period_bytes_max   = 4*1024,
	.periods_min        = 1,
	.periods_max        = 1024,
};

static int nuc970_dma_hw_params(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&nuc970_audio->irqlock, flags);

	if(runtime->dma_addr == 0) {
		ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
		if (ret < 0)
			return ret;
		nuc970_audio->substream[substream->stream] = substream;
	}

	nuc970_audio->dma_addr[substream->stream] = runtime->dma_addr | 0x80000000;
	nuc970_audio->buffersize[substream->stream] = params_buffer_bytes(params);

	spin_unlock_irqrestore(&nuc970_audio->irqlock, flags);

	return ret;
}

static void nuc970_update_dma_register(struct snd_pcm_substream *substream,
				dma_addr_t dma_addr, size_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	void __iomem *mmio_addr, *mmio_len;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mmio_addr = nuc970_audio->mmio + ACTL_PDESB;
		mmio_len = nuc970_audio->mmio + ACTL_PDES_LENGTH;
	} else {
		mmio_addr = nuc970_audio->mmio + ACTL_RDESB;
		mmio_len = nuc970_audio->mmio + ACTL_RDES_LENGTH;
	}

	AUDIO_WRITE(mmio_addr, dma_addr);
	AUDIO_WRITE(mmio_len, count);
}

static irqreturn_t nuc970_dma_interrupt(int irq, void *dev_id)
{
	struct nuc970_audio *nuc970_audio = dev_id;
	unsigned long val;
	unsigned long flags;
	int stream;

	spin_lock_irqsave(&nuc970_audio->irqlock, flags);

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);

	if (val & R_DMA_IRQ) {
		stream = SNDRV_PCM_STREAM_CAPTURE;
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val | R_DMA_IRQ);

		val = AUDIO_READ(nuc970_audio->mmio + ACTL_RSR);

		if (val & R_DMA_RIA_IRQ) {
			val = R_DMA_RIA_IRQ;
			AUDIO_WRITE(nuc970_audio->mmio + ACTL_RSR, val);
		}

	} else if (val & P_DMA_IRQ) {
		stream = SNDRV_PCM_STREAM_PLAYBACK;
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val | P_DMA_IRQ);

		val = AUDIO_READ(nuc970_audio->mmio + ACTL_PSR);

		if (val & P_DMA_RIA_IRQ) {
			val = P_DMA_RIA_IRQ;
			AUDIO_WRITE(nuc970_audio->mmio + ACTL_PSR, val);
		}

	} else {
		dev_err(nuc970_audio->dev, "Wrong DMA interrupt status!\n");
		spin_unlock_irqrestore(&nuc970_audio->irqlock, flags);
		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&nuc970_audio->irqlock, flags);

	snd_pcm_period_elapsed(nuc970_audio->substream[stream]);

	return IRQ_HANDLED;
}

static int nuc970_dma_hw_free(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;

	snd_pcm_lib_free_pages(substream);
	nuc970_audio->substream[substream->stream] = NULL;
	return 0;
}

static int nuc970_dma_prepare(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long flags, val;
	int ret = 0;

	spin_lock_irqsave(&nuc970_audio->irqlock, flags);

	nuc970_update_dma_register(substream,
				nuc970_audio->dma_addr[substream->stream],
				nuc970_audio->buffersize[substream->stream]);

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_RESET);

	switch (runtime->channels) {
	case 1:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val &= ~(PLAY_LEFT_CHNNEL | PLAY_RIGHT_CHNNEL);
			val |= PLAY_RIGHT_CHNNEL;
			val |= (PLAY_LEFT_CHNNEL | PLAY_RIGHT_CHNNEL);
		} else {
			val &= ~(RECORD_LEFT_CHNNEL | RECORD_RIGHT_CHNNEL);
			val |= RECORD_LEFT_CHNNEL;
		}
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
		break;
	case 2:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= (PLAY_LEFT_CHNNEL | PLAY_RIGHT_CHNNEL);
		else
			val |= (RECORD_LEFT_CHNNEL | RECORD_RIGHT_CHNNEL);
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
		break;
	default:
		ret = -EINVAL;
	}

	/* set DMA IRQ to half */
	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);
	val &= ~0xf000;
	val |= (R_DMA_IRQ_SEL_HALF | P_DMA_IRQ_SEL_HALF);
	AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val);

	spin_unlock_irqrestore(&nuc970_audio->irqlock, flags);
	return ret;
}

static int nuc970_dma_getposition(struct snd_pcm_substream *substream,
					dma_addr_t *src, dma_addr_t *dst)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;

	if (src != NULL)
		*src = AUDIO_READ(nuc970_audio->mmio + ACTL_PDESC);

	if (dst != NULL)
		*dst = AUDIO_READ(nuc970_audio->mmio + ACTL_RDESC);

	return 0;
}

static snd_pcm_uframes_t nuc970_dma_pointer(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t src, dst;
	unsigned long res;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	snd_pcm_uframes_t frames;

	spin_lock(&nuc970_audio->lock);

	nuc970_dma_getposition(substream, &src, &dst);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		res = dst - runtime->dma_addr;
	else
		res = src - runtime->dma_addr;

	 frames = bytes_to_frames(substream->runtime, res);

	 spin_unlock(&nuc970_audio->lock);

	 return frames;
}

static int nuc970_dma_open(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio;

	snd_soc_set_runtime_hwparams(substream, &nuc970_pcm_hardware);
	nuc970_audio = nuc970_i2s_data;
	runtime->private_data = nuc970_audio;

	return 0;
}

static int nuc970_dma_close(struct snd_soc_component *component,
		     struct snd_pcm_substream *substream)
{
	return 0;
}

int nuc970_dma_create(struct nuc970_audio *nuc970_audio)
{
	int ret = request_irq(nuc970_audio->irq_num, nuc970_dma_interrupt, 0, "nuc970-dma", nuc970_audio);

	if(ret)
		return -EBUSY;

	return ret;
}
EXPORT_SYMBOL_GPL(nuc970_dma_create);

int nuc970_dma_destroy(struct nuc970_audio *nuc970_audio)
{
	 free_irq(nuc970_audio->irq_num, nuc970_audio);

	 return 0;
}
EXPORT_SYMBOL_GPL(nuc970_dma_destroy);

static int nuc970_dma_mmap(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream,
		    struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start,
			       substream->runtime->dma_addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static void nuc970_dma_free_dma_buffers(struct snd_soc_component *component,
			     struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static u64 nuc970_pcm_dmamask = DMA_BIT_MASK(32);
static int nuc970_dma_new(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &nuc970_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
		card->dev, 4 * 1024, (4 * 1024) - 1);

	return 0;
}

struct snd_soc_component_driver nuc970_asrc_component = {
	.name		= DRV_NAME,
	.pcm_construct	= nuc970_dma_new,
	.pcm_destruct	= nuc970_dma_free_dma_buffers,
	.open		= nuc970_dma_open,
	.close		= nuc970_dma_close,
	.hw_params	= nuc970_dma_hw_params,
	.hw_free	= nuc970_dma_hw_free,
	.prepare	= nuc970_dma_prepare,
	.pointer	= nuc970_dma_pointer,
	.mmap		= nuc970_dma_mmap,
};


static int nuc970_soc_platform_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev, &nuc970_asrc_component,
					       NULL, 0);
}

static int nuc970_soc_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id nuc970_audio_pcm_of_match[] = {
	{   .compatible = "nuvoton,nuc970-audio-pcm"    },
	{   },
};
MODULE_DEVICE_TABLE(of, nuc970_audio_pcm_of_match);

static struct platform_driver nuc970_pcm_driver = {
	.driver = {
			.name = "nuc970-audio-pcm",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(nuc970_audio_pcm_of_match),
	},

	.probe = nuc970_soc_platform_probe,
	.remove = nuc970_soc_platform_remove,
};

module_platform_driver(nuc970_pcm_driver);

MODULE_DESCRIPTION("nuc970 Audio DMA module");
MODULE_LICENSE("GPL");
