/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_virtual_default"
//#define LOG_NDEBUG 0

#include <errno.h>
#include <malloc.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>


#include <log/log.h>

#include <hardware/audio.h>
#include <hardware/hardware.h>
#include <system/audio.h>

#define VIRTUAL_DEFAULT_SAMPLE_RATE   8000
#define VIRTUAL_DEFAULT_AUDIO_FORMAT  AUDIO_FORMAT_PCM_16_BIT

#define VIRTUAL_INPUT_BUFFER_MILLISECONDS  20
#define VIRTUAL_INPUT_DEFAULT_CHANNEL_MASK AUDIO_CHANNEL_IN_MONO

#define VIRTUAL_OUTPUT_BUFFER_MILLISECONDS  20
#define VIRTUAL_OUTPUT_DEFAULT_CHANNEL_MASK AUDIO_CHANNEL_OUT_MONO


#define AUDIO_FIFO_C	"/data/vendor/audio_virtual/audio_fifo_c"
#define AUDIO_FIFO_P	"/data/vendor/audio_virtual/audio_fifo_p"

#define MAX_BUF_LEN		256


static int audio_fifo_capture =-1;
static int audio_fifo_playback = -1;


#define MAX_PACKET_LEN (165*128)


struct virtual_audio_device {
    struct audio_hw_device device;
	pthread_mutex_t lock; /* see note below on mutex acquisition order */
};

struct virtual_stream_out {
    struct audio_stream_out stream;
    int64_t last_write_time_us;
    uint32_t sample_rate;
    audio_channel_mask_t channel_mask;
    audio_format_t format;
    size_t frame_count;

	bool standby;

	struct virtual_audio_device *adev;           /* hardware information - only using this for the lock */
};

struct virtual_stream_in {
    struct audio_stream_in stream;
    int64_t last_read_time_us;
    uint32_t sample_rate;
    audio_channel_mask_t channel_mask;
    audio_format_t format;
    size_t frame_count;

	bool standby;

	struct virtual_audio_device *adev;           /* hardware information - only using this for the lock */
};



static int openin_readAudioPipe()//playback for stm32 read from audio cards.
{
	int fd_fifo;
	//ALOGE("openin_readAudioPipe()");
	fd_fifo = open(AUDIO_FIFO_C,O_RDONLY | O_NONBLOCK);
	if(fd_fifo < 0){
		ALOGE("openin_readAudioPipe() pipe %s: open failed,errno=%d", AUDIO_FIFO_C,errno);
		return -1;
	}else{
		return fd_fifo;
	}
}


static int openout_writeAudioPipe()//recorder for stm32 write to audio cards.
{
	int fd_fifo;
	//ALOGE("openout_writeAudioPipe()");
	fd_fifo = open(AUDIO_FIFO_P,O_WRONLY | O_NONBLOCK);
	if(fd_fifo < 0){
		ALOGE("openout_writeAudioPipe() pipe %s: open failed,errno=%d", AUDIO_FIFO_P,errno);
		return -1;
	}else{
		return fd_fifo;
	}
}


static int closein_readAudioPipe()
{
	//ALOGE("closein_readAudioPipe()");

	if(audio_fifo_capture >= 0){
		close(audio_fifo_capture);
	}

	return 0;
}


static int closeout_writeAudioPipe()
{
	//ALOGE("closeout_writeAudioPipe()");

	if(audio_fifo_playback >= 0){
		close(audio_fifo_playback);
	}
	return 0;
}


static int out_writeAudioPipe(const void *pszBuf,ssize_t len)
{
	//unsigned char cmd_temp[MAX_PACKET_LEN] = {0};
	//memcpy(cmd_temp,pszBuf,len);
	int ret = -1;
	const void * write_buff = pszBuf;

	if(audio_fifo_playback >= 0){
		ret = write(audio_fifo_playback,write_buff,len);
		if(ret < 0){
			ALOGE("out_writeAudioPipe() failed errno = %d",errno);
		}
	}else{
		audio_fifo_playback= openout_writeAudioPipe();
	}
	ALOGE("out_writeAudioPipe() ret =%d",ret);
	return ret;
}

static int in_readAudioPipe(void *pszBuf,ssize_t len)
{
	//unsigned char cmd_temp[MAX_PACKET_LEN] = {0};
	void * read_buff = pszBuf;
	int ret = -1;

	if(audio_fifo_capture >= 0){
		ret = read(audio_fifo_capture,read_buff,len);
		if(ret > 0){
			//memcpy(pszBuf,cmd_temp,ret);
		}else if(ret < 0){
			ALOGE("in_readAudioPipe() failed errno = %d",errno);
		}
	}else{
		audio_fifo_capture = openin_readAudioPipe();
	}
	ALOGE("in_readAudioPipe() ret =%d",ret);
	return ret;
}


static int initAudioPipe()
{
	char Cmdbuf[MAX_BUF_LEN] = {0};
	if(mkfifo(AUDIO_FIFO_C,0777) != 0){
		if(errno == EEXIST){
			ALOGE("initAudioPipe() pipe %s: exist", AUDIO_FIFO_C);
		}else{
			ALOGE("initAudioPipe() pipe %s: mkfifo failed errno:%d", AUDIO_FIFO_C,errno);
			//return -1;
		}
	}

	if(mkfifo(AUDIO_FIFO_P,0777) != 0){
		if(errno == EEXIST){
			ALOGE("initAudioPipe() pipe %s: exist", AUDIO_FIFO_P);
		}else{
			ALOGE("initAudioPipe() pipe %s: mkfifo failed errno:%d", AUDIO_FIFO_P,errno);
			return -1;
		}
	}

	sprintf(Cmdbuf,"chmod 777 %s",AUDIO_FIFO_C);
	system(Cmdbuf);

	memset(Cmdbuf,0x00,MAX_BUF_LEN);
	
	sprintf(Cmdbuf,"chmod 777 %s",AUDIO_FIFO_P);
	system(Cmdbuf);
	
	return 0;
}

static void device_lock(struct virtual_audio_device *adev) {
    pthread_mutex_lock(&adev->lock);
}

static void device_unlock(struct virtual_audio_device *adev) {
    pthread_mutex_unlock(&adev->lock);
}


static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    const struct virtual_stream_out *out = (const struct virtual_stream_out *)stream;

    //ALOGE("out_get_sample_rate: %u", out->sample_rate);
    return out->sample_rate;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    //struct virtual_stream_out *out = (struct virtual_stream_out *)stream;

    //ALOGE("out_set_sample_rate: %d", rate);
    //out->sample_rate = rate;
    return 0;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    const struct virtual_stream_out *out = (const struct virtual_stream_out *)stream;
    size_t buffer_size = out->frame_count *
                         audio_stream_out_frame_size(&out->stream);

    //ALOGE("out_get_buffer_size: %zu", buffer_size);
    return buffer_size;
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    const struct virtual_stream_out *out = (const struct virtual_stream_out *)stream;

    //ALOGE("out_get_channels: %x", out->channel_mask);
    return out->channel_mask;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    const struct virtual_stream_out *out = (const struct virtual_stream_out *)stream;

    //ALOGE("out_get_format: %d", out->format);
    return out->format;
}

static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    //struct virtual_stream_out *out = (struct virtual_stream_out *)stream;

    //ALOGE("out_set_format: %d", format);
    //out->format = format;
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    //ALOGE("out_standby");
	struct virtual_stream_out *out = (struct virtual_stream_out *)stream;
    //out->last_write_time_us = 0; unnecessary as a stale write time has same effect
    out->standby = true;
    return 0;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    //ALOGE("out_dump");
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    ALOGE("out_set_parameters kvpairs= %s",kvpairs);
    return 0;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    ALOGE("out_get_parameters keys =%s",keys);
    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    ALOGE("out_get_latency");
    return VIRTUAL_OUTPUT_BUFFER_MILLISECONDS;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    //ALOGE("out_set_volume: Left:%f Right:%f", left, right);
    return 0;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    ALOGE("out_write: bytes: %zu", bytes);

    /* XXX: fake timing for audio output */
    struct virtual_stream_out *out = (struct virtual_stream_out *)stream;
	struct virtual_audio_device *adev = out->adev;

	size_t write_len = -1;
	
    struct timespec t = { .tv_sec = 0, .tv_nsec = 0 };
    clock_gettime(CLOCK_MONOTONIC, &t);
    const int64_t now = (t.tv_sec * 1000000000LL + t.tv_nsec) / 1000;
    const int64_t elapsed_time_since_last_write = now - out->last_write_time_us;
    int64_t sleep_time = bytes * 1000000LL / audio_stream_out_frame_size(stream) /
               out_get_sample_rate(&stream->common) - elapsed_time_since_last_write;
    if (sleep_time > 0) {
        usleep(sleep_time);
    } else {
        // we don't sleep when we exit standby (this is typical for a real alsa buffer).
        sleep_time = 0;
    }
    out->last_write_time_us = now + sleep_time;
    // last_write_time_us is an approximation of when the (simulated) alsa
    // buffer is believed completely full. The usleep above waits for more space
    // in the buffer, but by the end of the sleep the buffer is considered
    // topped-off.
    //
    // On the subsequent out_write(), we measure the elapsed time spent in
    // the mixer. This is subtracted from the sleep estimate based on frames,
    // thereby accounting for drain in the alsa buffer during mixing.
    // This is a crude approximation; we don't handle underruns precisely.

	if(out->standby == true){
		device_lock(adev);
		audio_fifo_playback = openout_writeAudioPipe();
		device_unlock(adev);
		
		if(audio_fifo_playback > 0){
			out->standby = false;
		}else{
			ALOGE("in_read() open in read audio pipe failed!!!");
			return -1;
		}
	}

	if(audio_fifo_playback > 0){
		device_lock(adev);
		write_len = out_writeAudioPipe(buffer,bytes);
		device_unlock(adev);
	}
	
	return write_len;
    //return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    *dsp_frames = 0;
    //ALOGE("out_get_render_position: dsp_frames: %p", dsp_frames);
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    //ALOGE("out_add_audio_effect: %p", effect);
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    //ALOGE("out_remove_audio_effect: %p", effect);
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
                                        int64_t *timestamp)
{
    *timestamp = 0;
    //ALOGE("out_get_next_write_timestamp: %ld", (long int)(*timestamp));
    return -EINVAL;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    const struct virtual_stream_in *in = (const struct virtual_stream_in *)stream;

    //ALOGE("in_get_sample_rate: %u", in->sample_rate);
    return in->sample_rate;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    //struct virtual_stream_in *in = (struct virtual_stream_in *)stream;

    //ALOGE("in_set_sample_rate: %u", rate);
    //in->sample_rate = rate;
    return 0;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    const struct virtual_stream_in *in = (const struct virtual_stream_in *)stream;
    size_t buffer_size = in->frame_count *
                         audio_stream_in_frame_size(&in->stream);

    //ALOGE("in_get_buffer_size: %zu", buffer_size);
    return buffer_size;
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    const struct virtual_stream_in *in = (const struct virtual_stream_in *)stream;

    //ALOGE("in_get_channels: %x", in->channel_mask);
    return in->channel_mask;
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    const struct virtual_stream_in *in = (const struct virtual_stream_in *)stream;

    //ALOGE("in_get_format: %d", in->format);
    return in->format;
}

static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    //struct virtual_stream_in *in = (struct virtual_stream_in *)stream;

    //ALOGE("in_set_format: %d", format);
    //in->format = format;
    return 0;
}

static int in_standby(struct audio_stream *stream)
{
    struct virtual_stream_in *in = (struct virtual_stream_in *)stream;
    in->last_read_time_us = 0;
	in->standby = true;
	//ALOGE("%s start!", __FUNCTION__);
    return 0;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
	ALOGE("%s start! kvpairs = %s", __FUNCTION__,kvpairs);
    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
	ALOGE("%s start! keys = %s", __FUNCTION__,keys);
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    ALOGE("in_read: bytes %zu", bytes);

    /* XXX: fake timing for audio input */
    struct virtual_stream_in *in = (struct virtual_stream_in *)stream;
	struct virtual_audio_device *adev = in->adev;

	size_t read_len = -1;
	
    struct timespec t = { .tv_sec = 0, .tv_nsec = 0 };
    clock_gettime(CLOCK_MONOTONIC, &t);
    const int64_t now = (t.tv_sec * 1000000000LL + t.tv_nsec) / 1000;

    // we do a full sleep when exiting standby.
    const bool standby = in->last_read_time_us == 0;
    const int64_t elapsed_time_since_last_read = standby ?
            0 : now - in->last_read_time_us;
    int64_t sleep_time = bytes * 1000000LL / audio_stream_in_frame_size(stream) /
            in_get_sample_rate(&stream->common) - elapsed_time_since_last_read;
    if (sleep_time > 0) {
        usleep(sleep_time);
    } else {
        sleep_time = 0;
    }
    in->last_read_time_us = now + sleep_time;
    // last_read_time_us is an approximation of when the (simulated) alsa
    // buffer is drained by the read, and is empty.
    //
    // On the subsequent in_read(), we measure the elapsed time spent in
    // the recording thread. This is subtracted from the sleep estimate based on frames,
    // thereby accounting for fill in the alsa buffer during the interim.

	if(in->standby == true){
		device_lock(adev);
		audio_fifo_capture = openin_readAudioPipe();
		device_unlock(adev);
		
		if(audio_fifo_capture > 0){
			in->standby = false;
		}else{
			ALOGE("in_read() open in read audio pipe failed!!!");
			return -1;
		}
	}

	if(audio_fifo_capture > 0){
		device_lock(adev);
		read_len = in_readAudioPipe(buffer,bytes);
		device_unlock(adev);
	}
    //memset(buffer, 0, bytes);
    return read_len;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
	//ALOGE("%s start!", __FUNCTION__);
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
	ALOGE("%s start!", __FUNCTION__);

    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
	ALOGE("%s start!", __FUNCTION__);

    return 0;
}

static size_t samples_per_milliseconds(size_t milliseconds,
                                       uint32_t sample_rate,
                                       size_t channel_count)
{
    return milliseconds * sample_rate * channel_count / 1000;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out,
                                   const char *address __unused)
{
    ALOGE("adev_open_output_stream...");

    *stream_out = NULL;
    struct virtual_stream_out *out =
            (struct virtual_stream_out *)calloc(1, sizeof(struct virtual_stream_out));
    if (!out)
        return -ENOMEM;

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;

	out->adev = (struct virtual_audio_device *)dev;
	
    //out->sample_rate = config->sample_rate;
    out->sample_rate = VIRTUAL_DEFAULT_SAMPLE_RATE;
    if (out->sample_rate == 0)
        out->sample_rate = VIRTUAL_DEFAULT_SAMPLE_RATE;
    //out->channel_mask = config->channel_mask;
    out->channel_mask = VIRTUAL_OUTPUT_DEFAULT_CHANNEL_MASK;
    if (out->channel_mask == AUDIO_CHANNEL_NONE)
        out->channel_mask = VIRTUAL_OUTPUT_DEFAULT_CHANNEL_MASK;
    //out->format = config->format;
    out->format = VIRTUAL_DEFAULT_AUDIO_FORMAT;
    if (out->format == AUDIO_FORMAT_DEFAULT)
        out->format = VIRTUAL_DEFAULT_AUDIO_FORMAT;
    out->frame_count = samples_per_milliseconds(
                           VIRTUAL_OUTPUT_BUFFER_MILLISECONDS,
                           out->sample_rate, 1);

    ALOGE("adev_open_output_stream: sample_rate: %u, channels: %x, format: %d,"
          " frames: %zu", out->sample_rate, out->channel_mask, out->format,
          out->frame_count);

	out->standby = true;
		  
    *stream_out = &out->stream;
    return 0;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    ALOGE("adev_close_output_stream...");
	struct virtual_audio_device *adev = (struct virtual_audio_device *)dev;
	device_lock(adev);
	closeout_writeAudioPipe();
	device_unlock(adev);
    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    ALOGE("adev_set_parameters() kvpairs=%s",kvpairs);
    return -ENOSYS;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    ALOGE("adev_get_parameters keys =%s",keys);
    return strdup("");
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    ALOGE("adev_init_check");
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    ALOGE("adev_set_voice_volume: %f", volume);
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    ALOGE("adev_set_master_volume: %f", volume);
    return -ENOSYS;
}

static int adev_get_master_volume(struct audio_hw_device *dev, float *volume)
{
    ALOGE("adev_get_master_volume: %f", *volume);
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev, bool muted)
{
    ALOGE("adev_set_master_mute: %d", muted);
    return -ENOSYS;
}

static int adev_get_master_mute(struct audio_hw_device *dev, bool *muted)
{
    ALOGE("adev_get_master_mute: %d", *muted);
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    ALOGE("adev_set_mode: %d", mode);
    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    ALOGE("adev_set_mic_mute: %d",state);
    return -ENOSYS;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    ALOGE("adev_get_mic_mute");
    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         const struct audio_config *config)
{
    size_t buffer_size = samples_per_milliseconds(
                             VIRTUAL_INPUT_BUFFER_MILLISECONDS,
                             config->sample_rate,
                             audio_channel_count_from_in_mask(
                                 config->channel_mask));

    if (!audio_has_proportional_frames(config->format)) {
        // Since the audio data is not proportional choose an arbitrary size for
        // the buffer.
        buffer_size *= 4;
    } else {
        buffer_size *= audio_bytes_per_sample(config->format);
    }
    //ALOGE("adev_get_input_buffer_size: %zu", buffer_size);
    return buffer_size;
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in,
                                  audio_input_flags_t flags __unused,
                                  const char *address __unused,
                                  audio_source_t source __unused)
{
    ALOGE("adev_open_input_stream...");

    *stream_in = NULL;
    struct virtual_stream_in *in = (struct virtual_stream_in *)calloc(1, sizeof(struct virtual_stream_in));
    if (!in)
        return -ENOMEM;

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

	in->adev = (struct virtual_audio_device *)dev;

    //in->sample_rate = config->sample_rate;
    in->sample_rate = VIRTUAL_DEFAULT_SAMPLE_RATE;
    if (in->sample_rate == 0)
        in->sample_rate = VIRTUAL_DEFAULT_SAMPLE_RATE;
    //in->channel_mask = config->channel_mask;
    in->channel_mask = VIRTUAL_INPUT_DEFAULT_CHANNEL_MASK;
    if (in->channel_mask == AUDIO_CHANNEL_NONE)
        in->channel_mask = VIRTUAL_INPUT_DEFAULT_CHANNEL_MASK;
    //in->format = config->format;
    in->format = VIRTUAL_DEFAULT_AUDIO_FORMAT;
    if (in->format == AUDIO_FORMAT_DEFAULT)
        in->format = VIRTUAL_DEFAULT_AUDIO_FORMAT;

	
    in->frame_count = samples_per_milliseconds(
                          VIRTUAL_INPUT_BUFFER_MILLISECONDS, in->sample_rate, 1);

    ALOGE("adev_open_input_stream: sample_rate: %u, channels: %x, format: %d,"
          "frames: %zu", in->sample_rate, in->channel_mask, in->format,
          in->frame_count);

	in->standby = true;
	
    *stream_in = &in->stream;
    return 0;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                   struct audio_stream_in *in)
{
    ALOGE("adev_close_input_stream...");
	struct virtual_audio_device *adev = (struct virtual_audio_device *)dev;
	device_lock(adev);
	closein_readAudioPipe();
	device_unlock(adev);
    return;
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    ALOGE("adev_dump");
    return 0;
}

static int adev_close(hw_device_t *device)
{
    ALOGE("adev_close");
    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    ALOGE("adev_open: %s", name);

    struct virtual_audio_device *adev;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct virtual_audio_device));
    if (!adev)
        return -ENOMEM;

    adev->device.common.tag = HARDWARE_DEVICE_TAG;
    adev->device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->device.common.module = (struct hw_module_t *) module;
    adev->device.common.close = adev_close;

    adev->device.init_check = adev_init_check;
    adev->device.set_voice_volume = adev_set_voice_volume;
    adev->device.set_master_volume = adev_set_master_volume;
    adev->device.get_master_volume = adev_get_master_volume;
    adev->device.set_master_mute = adev_set_master_mute;
    adev->device.get_master_mute = adev_get_master_mute;
    adev->device.set_mode = adev_set_mode;
    adev->device.set_mic_mute = adev_set_mic_mute;
    adev->device.get_mic_mute = adev_get_mic_mute;
    adev->device.set_parameters = adev_set_parameters;
    adev->device.get_parameters = adev_get_parameters;
    adev->device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->device.open_output_stream = adev_open_output_stream;
    adev->device.close_output_stream = adev_close_output_stream;
    adev->device.open_input_stream = adev_open_input_stream;
    adev->device.close_input_stream = adev_close_input_stream;
    adev->device.dump = adev_dump;
	
	initAudioPipe();

    *device = &adev->device.common;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "Virtual audio HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
