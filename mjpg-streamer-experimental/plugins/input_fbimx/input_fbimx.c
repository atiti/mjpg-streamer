#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>
#include <linux/fb.h>
#include <fcntl.h>
#include "imxvpuapi/imxvpuapi_jpeg.h"

#include "ipu_csc.h"

#include "../../mjpg_streamer.h"
#include "../../utils.h"

#define INPUT_PLUGIN_NAME "FB streamer on i.MX6"

/* private functions and variables to this plugin */
static pthread_t   worker;
static globals     *pglobal;
static pthread_mutex_t controls_mutex;
static int plugin_number;
/* JPEG Encoder related globals */
ImxVpuEncReturnCodes enc_ret;
uint8_t *mapped_virtual_address = NULL;
ImxVpuJPEGEncoder *jpeg_encoder = NULL;
ImxVpuFramebuffer framebuffer;
ImxVpuJPEGEncParams enc_params;
void *acquired_handle;
size_t output_buffer_size;


void *worker_thread(void *);
void worker_cleanup(void *);
void help(void);

static int delay = 30;

/* details of converted JPG pictures */
struct pic {
    const unsigned char *data;
    const int size;
};

/*** plugin interface functions ***/

/******************************************************************************
Description.: parse input parameters
Input Value.: param contains the command line string and a pointer to globals
Return Value: 0 if everything is ok
******************************************************************************/
int input_init(input_parameter *param, int plugin_no)
{
    int i;

    if(pthread_mutex_init(&controls_mutex, NULL) != 0) {
        IPRINT("could not initialize mutex variable\n");
        exit(EXIT_FAILURE);
    }

    param->argv[0] = INPUT_PLUGIN_NAME;

    /* show all parameters for DBG purposes */
    for(i = 0; i < param->argc; i++) {
        DBG("argv[%d]=%s\n", i, param->argv[i]);
    }

    reset_getopt();
    while(1) {
        int option_index = 0, c = 0;
        static struct option long_options[] = {
            {"h", no_argument, 0, 0
            },
            {"help", no_argument, 0, 0},
            {"d", required_argument, 0, 0},
            {"delay", required_argument, 0, 0},
            {"r", required_argument, 0, 0},
            {"resolution", required_argument, 0, 0},
            {0, 0, 0, 0}
        };

        c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help();
            return 1;
        }

        switch(option_index) {
            /* h, help */
        case 0:
        case 1:
            DBG("case 0,1\n");
            help();
            return 1;
            break;

            /* d, delay */
        case 2:
        case 3:
            DBG("case 2,3\n");
            delay = atoi(optarg);
            break;

            /* r, resolution */
        case 4:
        case 5:
            DBG("case 4,5\n");
            break;

        default:
            DBG("default case\n");
            help();
            return 1;
        }
    }

    pglobal = param->global;

    IPRINT("delay.............: %i\n", delay);

    return 0;
}

/******************************************************************************
Description.: stops the execution of the worker thread
Input Value.: -
Return Value: 0
******************************************************************************/
int input_stop(int id)
{
    DBG("will cancel input thread\n");
    pthread_cancel(worker);

    return 0;
}

/******************************************************************************
Description.: starts the worker thread and allocates memory
Input Value.: -
Return Value: 0
******************************************************************************/
int input_run(int id)
{
    pglobal->in[id].buf = NULL;

    if(pthread_create(&worker, 0, worker_thread, NULL) != 0) {
        free(pglobal->in[id].buf);
        fprintf(stderr, "could not start worker thread\n");
        exit(EXIT_FAILURE);
    }
    pthread_detach(worker);

    return 0;
}

/******************************************************************************
Description.: print help message
Input Value.: -
Return Value: -
******************************************************************************/
void help(void)
{
    fprintf(stderr, " ---------------------------------------------------------------\n" \
    " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
    " ---------------------------------------------------------------\n" \
    " The following parameters can be passed to this plugin:\n\n" \
    " [-d | --delay ]........: delay to pause between frames\n" \
    " [-r | --resolution]....: can be 960x720, 640x480, 320x240, 160x120\n"
    " ---------------------------------------------------------------\n");
}

void *acquire_output_buffer(void *context, size_t size, void **acquired_handle) {
    void *mem;
    ((void)(context));
    mem = malloc(size);
    *acquired_handle = mem;
    return mem;
}

void finish_output_buffer(void *context, void *acquired_handle) {
    ((void)(context));
    ((void *)(acquired_handle));
}

void setup_jpeg_enc_context(int width, int height, int depth) {
    memset(&framebuffer, 0, sizeof(framebuffer));
    framebuffer.y_stride = width;
    framebuffer.cbcr_stride = width / 2;
    framebuffer.y_offset = 0;
    framebuffer.cb_offset = width * height;
    framebuffer.cr_offset = (width * height) + ((width * height) / 4);
}

/******************************************************************************
Description.: copy a picture from testpictures.h and signal this to all output
              plugins, afterwards switch to the next frame of the animation.
Input Value.: arg is not used
Return Value: NULL
******************************************************************************/
void *worker_thread(void *arg)
{
    int i = 0;
    int fb = 0;
    int res_changed = 0;
    int depth=0, width=0, height=0, size_to_read=0;
    struct fb_var_screeninfo fb_varinfo;
    void *raw_buffer = NULL;
    ipu_csc_t csc;
    ipu_csc_format_t input_format = { 1024, 768, 24, V4L2_PIX_FMT_RGB32 };
    ipu_csc_format_t output_format = { 1024, 768, 16, V4L2_PIX_FMT_YUV420 };

    memset(&fb_varinfo, 0, sizeof(struct fb_var_screeninfo));

    /* set cleanup handler to cleanup allocated resources */
    pthread_cleanup_push(worker_cleanup, NULL);

    /* Open the framebuffer */
    fb = open("/dev/fb0", O_RDONLY);
    if (fb == -1) return NULL;

    /* Initialize jpeg encoder */
    imx_vpu_jpeg_enc_open(&jpeg_encoder, NULL);


    while(!pglobal->stop) {

        /* copy JPG picture to global buffer */
        pthread_mutex_lock(&pglobal->in[plugin_number].db);

	/* Read framebuffer info */
	ioctl(fb, FBIOGET_VSCREENINFO, &fb_varinfo);
	if (width != fb_varinfo.xres ||
	    height != fb_varinfo.yres) {
	    res_changed = 1;
	}
	width = fb_varinfo.xres;
	height = fb_varinfo.yres;
	depth = fb_varinfo.bits_per_pixel;
	size_to_read = width*height*(depth/8);

	switch (depth) {
	    case 32:
	        input_format.fmt = V4L2_PIX_FMT_BGR32;
		break;
	    case 24:
		input_format.fmt = V4L2_PIX_FMT_BGR24;
		break;
	    case 16:
		input_format.fmt = V4L2_PIX_FMT_RGB565;
		break;
	    default:
		fprintf(stderr, "Depth not supported: %d\n", depth);
		break;
	};

	/* Setup color space conversion from RGB -> YUV */
	output_format.width = width/2;
	output_format.height = height/2;
	input_format.width = width;
	input_format.height = height;
	input_format.bpp = depth;

	if (res_changed) {
	    setup_jpeg_enc_context(width/2, height/2, depth);

	    fprintf(stderr, "Resolution change detected.\n");
	    if (ipu_csc_init(&csc, &input_format, &output_format) < 0) {
	        fprintf(stderr, "ipu csc init failed\n");
	    }
	    if (raw_buffer)
		free(raw_buffer);
	    raw_buffer = malloc(size_to_read);

	    if (framebuffer.dma_buffer) imx_vpu_dma_buffer_deallocate(framebuffer.dma_buffer);
	    framebuffer.dma_buffer = imx_vpu_dma_buffer_allocate(imx_vpu_enc_get_default_allocator(), width*height*2, 1, 0);
	    if (framebuffer.dma_buffer == NULL) {
	        fprintf(stderr, "could not allocate DMA buffer for input fb\n");
	    }
	    res_changed = 0;
	}
	
	/* Read the frame from the fb */
	lseek(fb, 0, SEEK_SET); 
	read(fb, raw_buffer, size_to_read);

	/* DMA buffer setup */
	mapped_virtual_address = imx_vpu_dma_buffer_map(framebuffer.dma_buffer, IMX_VPU_MAPPING_FLAG_WRITE);
	/* Do colorscapce conversion */
	if (ipu_csc_convert(&csc, raw_buffer, mapped_virtual_address) < 0) {
	    fprintf(stderr, "Failed to do csc\n");
	}	

	/* Unmap DMA */
	imx_vpu_dma_buffer_unmap(framebuffer.dma_buffer);
	
	/* Encode */
	memset(&enc_params, 0, sizeof(enc_params));
	enc_params.frame_width = width/2;
	enc_params.frame_height = height/2;
	enc_params.quality_factor = 50;
	enc_params.color_format = IMX_VPU_COLOR_FORMAT_YUV420;
	enc_params.acquire_output_buffer = acquire_output_buffer;
	enc_params.finish_output_buffer = finish_output_buffer;
	enc_params.output_buffer_context = NULL;

	if (acquired_handle) {
	    free(acquired_handle);
 	    acquired_handle = NULL;
	}

	enc_ret = imx_vpu_jpeg_enc_encode(jpeg_encoder, &framebuffer, &enc_params, &acquired_handle, &output_buffer_size);


	if (enc_ret != IMX_VPU_ENC_RETURN_CODE_OK) {
	    fprintf(stderr, "could not encode this image: %s\n", imx_vpu_enc_error_string(enc_ret));
	}	

	pglobal->in[plugin_number].size = output_buffer_size;
	//memcpy(pglobal->in[plugin_number].buf, acquired_handle, output_buffer_size);
	pglobal->in[plugin_number].buf = acquired_handle;

	//if (acquired_handle != NULL) {
	//    free(acquired_handle);
	//    acquired_handle = NULL;
	//}
	//fprintf(stderr, "all good until here 8\n");
	
        /* signal fresh_frame */
        pthread_cond_broadcast(&pglobal->in[plugin_number].db_update);
        pthread_mutex_unlock(&pglobal->in[plugin_number].db);

        usleep(1000 * delay);
    }

    IPRINT("leaving input thread, calling cleanup function now\n");
    close(fb);
    imx_vpu_dma_buffer_deallocate(framebuffer.dma_buffer);
    imx_vpu_jpeg_enc_close(jpeg_encoder);
    jpeg_encoder = NULL;

    free(raw_buffer);

    pthread_cleanup_pop(1);

    return NULL;
}

/******************************************************************************
Description.: this functions cleans up allocated resources
Input Value.: arg is unused
Return Value: -
******************************************************************************/
void worker_cleanup(void *arg)
{
    static unsigned char first_run = 1;

    if(!first_run) {
        DBG("already cleaned up resources\n");
        return;
    }

    first_run = 0;
    DBG("cleaning up resources allocated by input thread\n");

    if(pglobal->in[plugin_number].buf != NULL) free(pglobal->in[plugin_number].buf);
}




