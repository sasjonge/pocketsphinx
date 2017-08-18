/* -*- c-basic-offset: 4; indent-tabs-mode: nil -*- */
/* ====================================================================
 * Copyright (c) 1999-2010 Carnegie Mellon University.  All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * This work was supported in part by funding from the Defense Advanced 
 * Research Projects Agency and the National Science Foundation of the 
 * United States of America, and the CMU Sphinx Speech Consortium.
 *
 * THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY ``AS IS'' AND 
 * ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * NOR ITS EMPLOYEES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ====================================================================
 *
 */
/*
 * continuous.c - Simple pocketsphinx command-line application to test
 *                both continuous listening/silence filtering from microphone
 *                and continuous file transcription.
 */

/*
 * This is a simple example of pocketsphinx application that uses continuous listening
 * with silence filtering to automatically segment a continuous stream of audio input
 * into utterances that are then decoded.
 * 
 * Remarks:
 *   - Each utterance is ended when a silence segment of at least 1 sec is recognized.
 *   - Single-threaded implementation for portability.
 *   - Uses audio library; can be replaced with an equivalent custom library.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>

#if defined(_WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#else
#include <sys/select.h>
#endif

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include "pocketsphinx.h"

#include <xmlrpc-c/base.h>
#include <xmlrpc-c/client.h>

#include <fdsink.h>

#define NAME "Xmlrpc-c Test Client"
#define VERSION "1.0"
#define NBTHREADS 1400
#define BEAMSIZE 1400

//global variables
static pthread_mutex_t mutex;
static int pip[NBTHREADS];
static int*  pipes[NBTHREADS];
int counter=0;
static char const *cfg;
static char* configDict[NBTHREADS];
static char* configLM[NBTHREADS];
static pthread_t threads[NBTHREADS];
static ps_decoder_t* psobj[NBTHREADS];
static cmd_ln_t* configobj[NBTHREADS];
static char* hypobj[NBTHREADS];
static int32 scobj[NBTHREADS];
static int32 firstscobj[BEAMSIZE];
static char* hmm;
static char* mllr;

//collecting parameter
static int INDEX;
static int PNBTHREADS;
static int PBEAMSIZE;
static int TRESHOLDY;
static char* HOST;
static int PORT;
static char* RPCPORT;
static char* DATAPATH;
static char* ASRCWD;
static char* HMM;
static char* MLLR;

static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    /* Argument file. */
    {"-argfile",
     ARG_STRING,
     NULL,
     "Argument file giving extra arguments."},
    {"-adcdev",
     ARG_STRING,
     NULL,
     "Name of audio device to use for input."},
    {"-infile",
     ARG_STRING,
     NULL,
     "Audio file to transcribe."},
    {"-inmic",
     ARG_BOOLEAN,
     "no",
     "Transcribe audio from microphone."},
    {"-time",
     ARG_BOOLEAN,
     "no",
     "Print word times in file transcription."},
    CMDLN_EMPTY_OPTION
};

static ps_decoder_t *ps;
static cmd_ln_t *config;
static FILE *rawfd;

//attributes
//boost::thread _gst_thread;
static GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode, *_tee;
static GstBus *_bus;
static int _bitrate, _channels, _depth, _sample_rate;
static GMainLoop *_loop;
static char* _format;
static guint8 *data; 
static int in[NBTHREADS];


static void setpipe(int pIn[]){
    int flags,i;
    for(i=0;i<PNBTHREADS;i++){
      in[i]=pIn[i];
      //flags = fcntl(this->in[i], F_GETFL, 0);
      //fcntl(this->in[i], F_SETFL, flags | O_NONBLOCK);
    }
}

//signal handling functions
static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
{
  GError *err;
  gchar *debug;

  gst_message_parse_error(message, &err, &debug);
  printf("\n\n%s\n\n",GST_MESSAGE_SRC_NAME(message));
  printf("\n\n%s\n\n","Error:");
  printf("\n\n%s\n\n",gst_message_type_get_name(GST_MESSAGE_TYPE(message)));
  
  g_error_free(err);
  g_free(debug);
  g_main_loop_quit(_loop);
  exitOnMainThread(1);
  return FALSE;
}

//audio stream
static void startAudiostream(int pip[])
{
  //set pipe
  setpipe(pip);
  //bit rate 24Khz=192bits/s
  _bitrate = 192;
  // Need to encoding or publish raw wave data
   _format="S16LE";
  // The bitrate at which to encode the audio
  _bitrate=192;
  // only available for raw data
  
  _channels=1;
  _depth=16;
  _sample_rate=16000;
  // The destination of the audio
  char* dst_type;
  dst_type="fdsink";
  // The source of the audio
  char* source_type;
  source_type="tcpserversrc";//tcpserversrc
  char* device;
  device="";
  //main loop
  _loop = g_main_loop_new(NULL, FALSE);
  //empty pipeline
  _pipeline = gst_pipeline_new("ros_pipeline");
  //pipe line bus
  _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
  //supervise bus
  gst_bus_add_signal_watch(_bus);
  //bus event handler
  g_signal_connect(_bus, "message::error",
                   G_CALLBACK(onMessage), 0);
  //free bus
  g_object_unref(_bus);
  // We create the sink first, just for convenience
  _sink = gst_element_factory_make("multifdsink", "sink");
 
 
  //tcpserversrc
  _source = gst_element_factory_make("tcpserversrc", "source");  
  g_object_set (G_OBJECT (_source), "host", "localhost",NULL);
  g_object_set (G_OBJECT (_source), "port","7000" ,NULL);
 

 
    gboolean link_ok;
   //filter
  _filter = gst_element_factory_make("capsfilter", "filter");
  //bus format
    GstCaps *caps;
    caps = gst_caps_new_simple("audio/x-raw",
                               "channels", G_TYPE_INT, _channels,
                               "width",    G_TYPE_INT, _depth,
                               "depth",    G_TYPE_INT, _depth,
                               "rate",     G_TYPE_INT, _sample_rate,
                               "signed",   G_TYPE_BOOLEAN, TRUE,
                               "format",   G_TYPE_STRING, "S16LE",
                               NULL);

    g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
    gst_caps_unref(caps);
    gst_bin_add_many( GST_BIN(_pipeline), _source,_filter, _sink, NULL);
    link_ok = gst_element_link_many( _source,_filter, _sink, NULL);
 
  //check if pipeline correctly linked
  if (!link_ok) {
    printf("\n%s\n","Unsupported media type.");
    exitOnMainThread(1);
  }
  //start the pipeline
  gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
  //streaming thread
  //adding descriptor to sink
  int i;
  for(i=0;i<PNBTHREADS;i++){
    g_signal_emit_by_name(_sink, "add", in[i], G_TYPE_NONE);
 }
  //start streaming
  //_gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
  pthread_t _gst_thread;
  pthread_create(&_gst_thread, NULL, g_main_loop_run, &_loop);

}
 //destructor
static void stopAudiostream()
{  int i;
  printf("\n%s\n","*******************STREAM TERMINATES*******************!!!.");
  //quit thread
  g_main_loop_quit(_loop);
  //stop pipeline
  gst_element_set_state(_pipeline, GST_STATE_NULL);
  //discharge the pipeline
  gst_object_unref(_pipeline);
  //discharge threads
  g_main_loop_unref(_loop);
 //close pipes
  for(i=0;i<PNBTHREADS;i++){
     close(in[i]);
  }
  printf("\n%s\n","*******************STREAM TERMINATED*******************!!!.");
}
//exit thread
void exitOnMainThread(int code)
{
  exit(code);
}

static void 
dieIfFaultOccurred (xmlrpc_env * const envP) {
    if (envP->fault_occurred) {
        fprintf(stderr, "ERROR: %s (%d)\n",
                envP->fault_string, envP->fault_code);
        exit(1);
    }
}

static void
print_word_times()
{
    int frame_rate = cmd_ln_int32_r(config, "-frate");
    ps_seg_t *iter = ps_seg_iter(ps);
    while (iter != NULL) {
        int32 sf, ef, pprob;
        float conf;

        ps_seg_frames(iter, &sf, &ef);
        pprob = ps_seg_prob(iter, NULL, NULL, NULL);
        conf = logmath_exp(ps_get_logmath(ps), pprob);
        printf("%s %.3f %.3f %f\n", ps_seg_word(iter), ((float)sf / frame_rate),
               ((float) ef / frame_rate), conf);
        iter = ps_seg_next(iter);
    }
}

static int
check_wav_header(char *header, int expected_sr)
{
    int sr;

    if (header[34] != 0x10) {
        E_ERROR("Input audio file has [%d] bits per sample instead of 16\n", header[34]);
        return 0;
    }
    if (header[20] != 0x1) {
        E_ERROR("Input audio file has compression [%d] and not required PCM\n", header[20]);
        return 0;
    }
    if (header[22] != 0x1) {
        E_ERROR("Input audio file has [%d] channels, expected single channel mono\n", header[22]);
        return 0;
    }
    sr = ((header[24] & 0xFF) | ((header[25] & 0xFF) << 8) | ((header[26] & 0xFF) << 16) | ((header[27] & 0xFF) << 24));
    if (sr != expected_sr) {
        E_ERROR("Input audio file has sample rate [%d], but decoder expects [%d]\n", sr, expected_sr);
        return 0;
    }
    return 1;
}

/*
 * Continuous recognition from a file
 */
static void
recognize_from_file()
{
    int16 adbuf[2048];
    const char *fname;
    const char *hyp;
    int32 k;
    uint8 utt_started, in_speech;
    int32 print_times = cmd_ln_boolean_r(config, "-time");

    fname = cmd_ln_str_r(config, "-infile");
    if ((rawfd = fopen(fname, "rb")) == NULL) {
        E_FATAL_SYSTEM("Failed to open file '%s' for reading",
                       fname);
    }
    
    if (strlen(fname) > 4 && strcmp(fname + strlen(fname) - 4, ".wav") == 0) {
        char waveheader[44];
	fread(waveheader, 1, 44, rawfd);
	if (!check_wav_header(waveheader, (int)cmd_ln_float32_r(config, "-samprate")))
    	    E_FATAL("Failed to process file '%s' due to format mismatch.\n", fname);
    }

    if (strlen(fname) > 4 && strcmp(fname + strlen(fname) - 4, ".mp3") == 0) {
	E_FATAL("Can not decode mp3 files, convert input file to WAV 16kHz 16-bit mono before decoding.\n");
    }
    
    ps_start_utt(ps);
    utt_started = FALSE;

    while ((k = fread(adbuf, sizeof(int16), 2048, rawfd)) > 0) {
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(ps);
        if (in_speech && !utt_started) {
            utt_started = TRUE;
        } 
        if (!in_speech && utt_started) {
            ps_end_utt(ps);
            hyp = ps_get_hyp(ps, NULL);
            if (hyp != NULL)
        	printf("%s\n", hyp);
            if (print_times)
        	print_word_times();
            fflush(stdout);

            ps_start_utt(ps);
            utt_started = FALSE;
        }
    }
    ps_end_utt(ps);
    if (utt_started) {
        hyp = ps_get_hyp(ps, NULL);
        if (hyp != NULL) {
    	    printf("%s\n", hyp);
    	    if (print_times) {
    		print_word_times();
	    }
	}
    }
    
    fclose(rawfd);
}

/* Sleep for specified msec */
static void
sleep_msec(int32 ms)
{
#if (defined(_WIN32) && !defined(GNUWINCE)) || defined(_WIN32_WCE)
    Sleep(ms);
#else
    /* ------------------- Unix ------------------ */
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
#endif
}


int  sendDataOut(char recognizedWord[]){
       /******************************RPC CLIENT*************************/

    xmlrpc_env env;
    xmlrpc_value * resultP;
    xmlrpc_int32 sum;
    char *serverUrl = malloc(strlen(HOST)+strlen(RPCPORT)+14);
    const char *  methodName = "on_word_recognized";
    strcpy(serverUrl, "http://");
    strcat(serverUrl, HOST);
    strcat(serverUrl, ":");
    strcat(serverUrl, RPCPORT);
    strcat(serverUrl, "/RPC2");
    
    /* Initialize our error-handling environment. */
    xmlrpc_env_init(&env);

    /* Create the global XML-RPC client object. */
    xmlrpc_client_init2(&env, XMLRPC_CLIENT_NO_FLAGS, NAME, VERSION, NULL, 0);
    dieIfFaultOccurred(&env);

    printf("Making XMLRPC call to server url '%s' method '%s' "
           "to request the sum "
           "of 5 and 7...\n", serverUrl, methodName);

    /* Make the remote procedure call */
    resultP = xmlrpc_client_call(&env, serverUrl, methodName,
                                 "(s)", recognizedWord);
    dieIfFaultOccurred(&env);
    
    /* Get our sum and print it out. */
    xmlrpc_read_int(&env, resultP, &sum);
    dieIfFaultOccurred(&env);
    printf("The sum is %d\n", sum);
    
    /* Dispose of our result value. */
    xmlrpc_DECREF(resultP);

    /* Clean up our error-handling environment. */
    xmlrpc_env_clean(&env);
    
    /* Shutdown our XML-RPC client library. */
    xmlrpc_client_cleanup();

    return sum;

/*******************************END******************************/


}

static int isEmpty(char* s){
       if(s==NULL)
         return 1;
       else{
          int n=strlen(s);
          int i;
          for(i=0;i<n;i++)
             if(s[i]!=' ')
               return 0;
          return 1;
       }
} 

/*
 * Main utterance processing loop:
 *     for (;;) {
 *        start utterance and wait for speech to process
 *        decoding till end-of-utterance silence will be detected
 *        print utterance result;
 *     }
 */
static void recognize_from_microphone(ps_decoder_t* ps, cmd_ln_t* config, int i)
{
    
    int16 adbuf[2048];
    uint8 utt_started, in_speech;
    int32 k,score, posterior;
    char const *hyp;
    int length,j,max,l,flags;
    char finalhyp[1000];
    finalhyp[0]='\0';
    int inout;//pipes
   
    if (ps_start_utt(ps) < 0)
        E_FATAL("Failed to start utterance\n");
    utt_started = FALSE;
    
     inout=pipes[i][0]; 
    //making non blocking stream
    flags = fcntl(inout, F_GETFL, 0);
    fcntl(inout, F_SETFL, flags | O_NONBLOCK);
    //audiostream output. only instanciated by thread 0
    if(i==0)
       startAudiostream(pip);
      
    E_INFO("Ready....\n");
    for (;;) {
       //printf("...........NOTHING %d %d....\n",i,k);
        k = read(inout, adbuf, 4096);
        if(k==-1)
           k++;
        if (k < 0)
            E_FATAL("Failed to read audio\n");
       k=k/2;
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(ps);
        if (in_speech && !utt_started) {
            utt_started = TRUE;
            E_INFO("Listening...\n");
        }
       
        if (!in_speech && utt_started) {
            /* speech -> silence transition, time to start new utterance  */
            ps_end_utt(ps);
            hyp = ps_get_hyp(ps, &score );
            posterior=ps_get_prob(ps);
            //print full result for this thread
            printf(" Final score: %d  \n", score);
            printf(" Final posterior: %d  \n", posterior);
            //discard previous hypothesis
            if(hypobj[i]!=NULL)
            free(hypobj[i]);
            if(hyp!=NULL){
                    //allocate space for new hypothesis
                    length=strlen(hyp)+1;
                    hypobj[i]=(char*)malloc(sizeof(char)*length);
                    //save hypothesis
                    strcpy(hypobj[i],hyp);
                    //save score;
                    scobj[i]=score; 
                    if(scobj[i]<TRESHOLDY) 
                    scobj[i]=0;
                    //print hyp
                    printf("\n\n HYPOTHESIS %d %d: %s \n\n",i,scobj[i], hyp);
            }else
                    scobj[i]=0;

            pthread_mutex_lock(&mutex);
            printf("\n\n------------------ THREAD %d,%d------------\n\n",i,counter);
            //critical section: shared ressource: counter
            counter++;
            if(counter==PNBTHREADS){
            //all threads have completed.
              counter=0;
            //get the BEAMSIZE best hypothesis
              for(l=0;l<PBEAMSIZE;l++){
                 max=0;
                 for(j=0;j<PNBTHREADS;j++)
                   if(scobj[j]>scobj[max] && scobj[j]<0 || scobj[j]<0 && scobj[max]>=0)
                     max=j;
                 firstscobj[l]=max;
                 if(scobj[max]<0)
                   scobj[max]=1;
              }
                 
              //merge the BEAMSIZE best hypothesis
              for(j=0;j<PBEAMSIZE;j++)
                 
                  if(scobj[firstscobj[j]]==1){
                          strcat(finalhyp,hypobj[firstscobj[j]]);
                          strcat(finalhyp," ");
                  }
              if(strlen(finalhyp)>0){
                if(isEmpty(finalhyp) != 0){
            printf("\n\n HYPOTHESIS: %s\n\n",finalhyp );
            if(sendDataOut(finalhyp)==0){
               printf("\n\n---HYPOTHESIS SENT SUCCESSFULLY-\n\n");
            }else
             printf("\n\n---HYPOTHESIS SENDING FAILED-\n\n");
           
                
              }else
                  printf("\n\n HYPOTHESIS: NOTHING\n\n");
               finalhyp[0]='\0';
            }}
            fflush(stdout);//output console  update
            printf("\n\n-----------------------------------------\n\n");
            //exit critical section
            pthread_mutex_unlock(&mutex);
            
            if (ps_start_utt(ps) < 0)
                E_FATAL("Failed to start utterance\n");
            utt_started = FALSE;
            E_INFO("Ready....\n");
        }
        sleep_msec(100);
    }
    close(inout);
}

void *recognizer(void*arg)
{



     int i;
     i=*((int*)arg);
      printf("\n%s\n","in");

     if(FALSE) {
        configobj[i]= cmd_ln_init(NULL,  cont_args_def,TRUE, "-dict", configDict[i],"-lm", configLM[i], "-inmic", "yes",NULL);
     }
      else {
        printf("\n%s\n",configDict[i]);
        configobj[i]= cmd_ln_init(NULL,  cont_args_def,TRUE, "-dict", configDict[i],"-lm", configLM[i], "-inmic", "yes",NULL);
      }

      printf("\n%s\n","out");
     /* Handle argument file as -argfile. */
    if (configobj[i] && (cfg = cmd_ln_str_r(configobj[i], "-argfile")) != NULL) {
        configobj[i] = cmd_ln_parse_file_r(configobj[i], cont_args_def, cfg, FALSE);
    }

    if (configobj[i] == NULL || (cmd_ln_str_r(configobj[i], "-infile") == NULL && cmd_ln_boolean_r(configobj[i], "-inmic") == FALSE)) {
  E_INFO("Specify '-infile <file.wav>' to recognize from file or '-inmic yes' to recognize from microphone.\n");
        cmd_ln_free_r(configobj[i]);
  return (void*)1;
    }

    ps_default_search_args(configobj[i]);
    psobj[i] = ps_init(configobj[i]);
    if (psobj[i] == NULL) {
        cmd_ln_free_r(configobj[i]);
        return (void*)1;
    }

  

    if (cmd_ln_str_r(configobj[i], "-infile") != NULL) {
        //recognize_from_file(psobj[i],configobj[i]);
    } else if (cmd_ln_boolean_r(configobj[i], "-inmic")) {
        recognize_from_microphone(psobj[i],configobj[i],i);
    }

    ps_free(psobj[i]);
    cmd_ln_free_r(configobj[i]);

    return (void*)0;

}

static unsigned numOfDigit(int n) {
  if (n <10) {
    return 1;
  }
  return 1 + numOfDigit(n/10);
}

/********************** MAIN THREADS ***************************************************/


int
main(int argc, char *argv[])
{
 //check all parameters are there
 if(argc!=12){
    printf("\n\n%s\n\n","Error in sphinx asr: incorrect parameters...");
   return 0;
 
 }else{

   //collecting parameter
    printf("\n%s\n","Loading parameters...");
   INDEX=atoi(argv[1]);
    printf("\n%s","INDEX: ");
    printf("%i\n",INDEX);
   PNBTHREADS=atoi(argv[2]);
    printf("\n%s","NBTHREADS: ");
    printf("%i\n",PNBTHREADS);
   PBEAMSIZE=atoi(argv[3]);
    printf("\n%s","BEAMSIZE: ");
    printf("%i\n",PBEAMSIZE);
   HOST=argv[4];
    printf("\n%s","LOCALHOST: ");
    printf("%s\n",HOST);
   PORT=atoi(argv[5]);
    printf("\n%s","LOCALPORT: ");
    printf("%i\n",PORT);
   DATAPATH=argv[6];
    printf("\n%s","DATAPATH: ");
    printf("%s\n",DATAPATH);
   ASRCWD="/home/sascha/suturo16/catkin_ws/src/pepper-dialog";
    printf("\n%s","ASRCWD: ");
    printf("%s\n",ASRCWD);
   RPCPORT="7000";
    printf("\n%s","RPCPORT: ");
    printf("%s\n",RPCPORT);
   TRESHOLDY=-10000;
    printf("\n%s","TRESHOLD: ");
    printf("%i\n",TRESHOLDY);
   HMM=argv[10];
    printf("\n%s","HMM: ");
    printf("%s\n",HMM);
   MLLR=argv[11];
    printf("\n%s","MLLR: ");
    printf("%s\n",MLLR);
   //create configuration data
   int i = 0;
   for(i=0;i<PNBTHREADS;i++){
      unsigned lengthOfIndexPlusI = numOfDigit(INDEX+i);
      char indexStr[12];
      sprintf(indexStr, "%d", (INDEX+i));
      configDict[i]=malloc(strlen(ASRCWD)+strlen(DATAPATH)+lengthOfIndexPlusI+13);
      strcpy(configDict[i], ASRCWD);
      strcat(configDict[i], "/");
      strcat(configDict[i], DATAPATH);
      strcat(configDict[i], "/pepper");
      strcat(configDict[i], indexStr);
      strcat(configDict[i], ".dic");
      printf("\n%s","DIC: ");
      printf("%s\n",configDict[i]);
      configLM[i]=malloc(strlen(ASRCWD)+strlen(DATAPATH)+lengthOfIndexPlusI+13);
      strcpy(configLM[i], ASRCWD);
      strcat(configLM[i], "/");
      strcat(configLM[i], DATAPATH);
      strcat(configLM[i], "/pepper");
      strcat(configLM[i], indexStr);
      strcat(configLM[i], ".lm");
      printf("\n%s","LM: ");
      printf("%s\n",configLM[i]);
   }
   
   hmm=malloc(strlen(ASRCWD)+strlen(HMM)+2);
    strcpy(hmm, ASRCWD);
    strcat(hmm, "/");
    strcat(hmm, HMM);
    mllr=malloc(strlen(ASRCWD)+strlen(MLLR)+2);
    strcpy(mllr, ASRCWD);
    strcat(mllr, "/");
    strcat(mllr, MLLR);

   int t=0,rc;
   strlen(NULL);
   //init gstreamer
   gst_init(&argc, &argv);
   //create pipes
   for(t=0;t<PNBTHREADS;t++){
      int intArr[2];
      pipes[t]=intArr;
      pipe(pipes[t]);
      pip[t]=pipes[t][1];
   }
   
   //create threads
   pthread_mutex_init(&mutex, NULL);
   for(t=0; t<PNBTHREADS; t++){
       printf("In main: creating thread %d\n", t);
       int myArray[t];
       rc = pthread_create(&threads[t], NULL, recognizer, (void *)(myArray));
       if (rc){
          printf("ERROR; return code from pthread_create() is %d\n", rc);
          exit(-1);
       }
    }

  //wait until all joining threads terminate
  pthread_exit(NULL);
  //destroys pipes
  for(t=0;t<PNBTHREADS;t++){
    close(pipes[t][0]);
    close(pipes[t][1]);
  }
  //destroy mutex
  pthread_mutex_destroy(&mutex);

  return 0;  
}
}