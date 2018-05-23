/* Copyright 2017 Streampunk Media Ltd.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include <nan.h>
#include "AudioIn.h"
#include "Persist.h"
#include "Params.h"
#include "ChunkQueue.h"
#include <mutex>
#include <condition_variable>
#include <map>
#include <portaudio.h>
#include <lame/lame.h>

using namespace v8;

namespace streampunk {

  // LAME MP3 BUFFER
  const int MP3_SIZE = 8820;
  unsigned char lameMp3Buffer[MP3_SIZE];
  short int pcmBigBuffer[MP3_SIZE];
  int BUFFER_FULL_AT = 10;
  int readPointer = 0;
  int bufferCounter = 0;
  int writeData = 0;

static std::map<char*, std::shared_ptr<Memory> > outstandingAllocs;
static void freeAllocCb(char* data, void* hint) {
  std::map<char*, std::shared_ptr<Memory> >::iterator it = outstandingAllocs.find(data);
  if (it != outstandingAllocs.end())
    outstandingAllocs.erase(it);
}

class InContext {
public:
  InContext(std::shared_ptr<AudioOptions> audioOptions, PaStreamCallback *cb)
    : mActive(true), mAudioOptions(audioOptions), mChunkQueue(mAudioOptions->maxQueue()) {

    PaError errCode = Pa_Initialize();
    if (errCode != paNoError) {
      std::string err = std::string("Could not initialize PortAudio: ") + Pa_GetErrorText(errCode);
      Nan::ThrowError(err.c_str());
    }

    printf("\n*** LAME VERSION ***\n");
    printf("Input %s\n", mAudioOptions->toString().c_str());

    PaStreamParameters inParams;
    memset(&inParams, 0, sizeof(PaStreamParameters));

    int32_t deviceID = (int32_t)mAudioOptions->deviceID();
    if ((deviceID >= 0) && (deviceID < Pa_GetDeviceCount()))
      inParams.device = (PaDeviceIndex)deviceID;
    else
      inParams.device = Pa_GetDefaultInputDevice();
    if (inParams.device == paNoDevice)
      Nan::ThrowError("No default input device");
    printf("Input device name is %s\n", Pa_GetDeviceInfo(inParams.device)->name);

    inParams.channelCount = mAudioOptions->channelCount();
    if (inParams.channelCount > Pa_GetDeviceInfo(inParams.device)->maxInputChannels)
      Nan::ThrowError("Channel count exceeds maximum number of input channels for device");

    uint32_t sampleFormat = mAudioOptions->sampleFormat();
    switch(sampleFormat) {
    case 8: inParams.sampleFormat = paInt8; break;
    case 16: inParams.sampleFormat = paInt16; break;
    case 24: inParams.sampleFormat = paInt24; break;
    case 32: inParams.sampleFormat = paInt32; break;
    default: Nan::ThrowError("Invalid sampleFormat");
    }

    inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultLowInputLatency;
    inParams.hostApiSpecificStreamInfo = NULL;

    double sampleRate = (double)mAudioOptions->sampleRate();
    uint32_t framesPerBuffer = paFramesPerBufferUnspecified;

    #ifdef __arm__
    framesPerBuffer = 256;
    inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultHighInputLatency;
    #endif

    errCode = Pa_OpenStream(&mStream, &inParams, NULL, sampleRate,
                            framesPerBuffer, paNoFlag, cb, this);
    if (errCode != paNoError) {
      std::string err = std::string("Could not open stream: ") + Pa_GetErrorText(errCode);
      Nan::ThrowError(err.c_str());
    }

    initLameEncoder();
  }

  ~InContext() {
    Pa_StopStream(mStream);
    Pa_Terminate();
  }

  void start() {
    PaError errCode = Pa_StartStream(mStream);
    if (errCode != paNoError) {
      std::string err = std::string("Could not start input stream: ") + Pa_GetErrorText(errCode);
      return Nan::ThrowError(err.c_str());
    }
  }

  void stop() {
    Pa_StopStream(mStream);
    Pa_Terminate();
  }

  std::shared_ptr<Memory> readChunk() {
    return mChunkQueue.dequeue();
  }

  int lameEncodeBuffer() {
    int writeCode = lame_encode_buffer_interleaved(lameMp3, pcmBigBuffer, MP3_SIZE / 2, lameMp3Buffer, MP3_SIZE);
    // printf("\nWrite code: %d\n", writeCode);

    bufferCounter = 0;
    readPointer = 0;
    if (writeCode == -1) printf("[LAME] ENCODING ERROR\n");
    return writeCode;
  }

  void lameCloseEncoding() {
    lame_encode_flush(lameMp3, lameMp3Buffer, MP3_SIZE);
    lame_close(lameMp3);
    // fclose(mp3File);
    printf("\n\n*** CLOSING FILE ***\n");
  }

  bool readBuffer(const void *srcBuf, uint32_t frameCount)
  {
    uint32_t bytesAvailable = frameCount * mAudioOptions->channelCount() * mAudioOptions->sampleFormat() / 8; // at the moment 1764
    short int *pcm_buffer = (short int *)srcBuf;

    memcpy(pcmBigBuffer + readPointer, pcm_buffer, frameCount * 2 * 2);

    readPointer += frameCount * 2; // 441 * 2
    bufferCounter++;

    if (bufferCounter == BUFFER_FULL_AT) writeData = lameEncodeBuffer(); // when the buffer is full then fire lame encoder
    else return 1;

    std::shared_ptr<Memory> dstBuf = Memory::makeNew(writeData);
    memcpy(dstBuf->buf(), (short int*) lameMp3Buffer, writeData);
    mChunkQueue.enqueue(dstBuf);
    return mActive;
  }

  void initLameEncoder()
  {
    if (lame_set_in_samplerate(lameMp3, 44100)) {
      return Nan::ThrowError("LAME set in samplerate failed");
    }
    if (lame_set_out_samplerate(lameMp3, 44100)) {
      return Nan::ThrowError("LAME set out samplerate failed");
    }
    /*
    if (lame_set_VBR(lameMp3, vbr_default)) {
      return Nan::ThrowError("LAME set VBR failed");
    } */
    if (lame_set_brate(lameMp3, 192)) {
      return Nan::ThrowError("LAME set bitrate failed");
    }
    if(lame_set_num_channels(lameMp3, 2) == -1) {
      return Nan::ThrowError("LAME set channels failed");
    }
    if(lame_set_mode(lameMp3, STEREO) == -1) {
      return Nan::ThrowError("LAME set mode failed");
    }
    // if (lame_set_disable_reservoir(lameMp3, TRUE) == -1) {
    //   return Nan::ThrowError("LAME disable reservoir failed");
    // }
    if(lame_set_quality(lameMp3, 2) == -1) {
      return Nan::ThrowError("LAME set quality failed");
    }
    lame_set_errorf(lameMp3, &lame_error);
    lame_set_debugf(lameMp3, &lame_debug);

    if(lame_init_params(lameMp3) == -1) {
      return Nan::ThrowError("LAME init params failed...");
    }
  }

  /* Callback functions for each type of lame message callback */
  static void lame_error(const char *fmt, va_list list)
  {
    printf("LAME ERROR: %s\n", fmt);
  }
  static void lame_debug(const char *fmt, va_list list)
  {
    printf("LAME DEB: %s\n", fmt);
  }

  void checkStatus(uint32_t statusFlags) {
    if (statusFlags) {
      std::string err = std::string("portAudio status - ");
      if (statusFlags & paInputUnderflow)
        err += "input underflow ";
      if (statusFlags & paInputOverflow)
        err += "input overflow ";

      std::lock_guard<std::mutex> lk(m);
      mErrStr = err;
    }
  }

  bool getErrStr(std::string& errStr) {
    std::lock_guard<std::mutex> lk(m);
    errStr = mErrStr;
    mErrStr = std::string();
    return errStr != std::string();
  }

  void quit() {
    std::unique_lock<std::mutex> lk(m);
    mActive = false;
    mChunkQueue.quit();
  }

private:
  bool mActive;
  std::shared_ptr<AudioOptions> mAudioOptions;
  ChunkQueue<std::shared_ptr<Memory> > mChunkQueue;
  PaStream* mStream;
  std::string mErrStr;
  mutable std::mutex m;
  std::condition_variable cv;

  // LAME
  lame_t lameMp3 = lame_init();
};

int InCallback(const void *input, void *output, unsigned long frameCount,
               const PaStreamCallbackTimeInfo *timeInfo,
               PaStreamCallbackFlags statusFlags, void *userData) {
  InContext *context = (InContext *)userData;
  context->checkStatus(statusFlags);
  return context->readBuffer(input, frameCount) ? paContinue : paComplete;
}

class InWorker : public Nan::AsyncWorker {
  public:
    InWorker(std::shared_ptr<InContext> InContext, Nan::Callback *callback)
      : AsyncWorker(callback), mInContext(InContext)
    { }
    ~InWorker() {}

    void Execute() {
      mInChunk = mInContext->readChunk();
    }

    void HandleOKCallback () {
      Nan::HandleScope scope;

      std::string errStr;
      if (mInContext->getErrStr(errStr)) {
        Local<Value> argv[] = { Nan::Error(errStr.c_str()) };
        callback->Call(1, argv, async_resource);
      }

      if (mInChunk) {
        outstandingAllocs.insert(make_pair((char*)mInChunk->buf(), mInChunk));
        Nan::MaybeLocal<Object> maybeBuf = Nan::NewBuffer((char*)mInChunk->buf(), mInChunk->numBytes(), freeAllocCb, 0);
        Local<Value> argv[] = { Nan::Null(), maybeBuf.ToLocalChecked() };
        callback->Call(2, argv, async_resource);
      } else {
        Local<Value> argv[] = { Nan::Null(), Nan::Null() };
        callback->Call(2, argv, async_resource);
      }
    }

  private:
    std::shared_ptr<InContext> mInContext;
    std::shared_ptr<Memory> mInChunk;
};

class QuitInWorker : public Nan::AsyncWorker {
  public:
    QuitInWorker(std::shared_ptr<InContext> InContext, Nan::Callback *callback)
      : AsyncWorker(callback), mInContext(InContext)
    { }
    ~QuitInWorker() {}

    void Execute() {
      mInContext->quit();
    }

    void HandleOKCallback () {
      Nan::HandleScope scope;
      mInContext->stop();
      callback->Call(0, NULL, async_resource);
    }

  private:
    std::shared_ptr<InContext> mInContext;
};

AudioIn::AudioIn(Local<Object> options) {
  mInContext = std::make_shared<InContext>(std::make_shared<AudioOptions>(options), InCallback);
}
AudioIn::~AudioIn() {}

void AudioIn::doStart() { mInContext->start(); }

NAN_METHOD(AudioIn::Start) {
  AudioIn* obj = Nan::ObjectWrap::Unwrap<AudioIn>(info.Holder());
  obj->doStart();
  info.GetReturnValue().SetUndefined();
}

NAN_METHOD(AudioIn::Read) {
  if (info.Length() != 2)
    return Nan::ThrowError("AudioIn Read expects 2 arguments");
  if (!info[0]->IsNumber())
    return Nan::ThrowError("AudioIn Read requires a valid advisory size as the first parameter");
  if (!info[1]->IsFunction())
    return Nan::ThrowError("AudioIn Read requires a valid callback as the second parameter");

  // uint32_t sizeAdv = Nan::To<uint32_t>(info[0]).FromJust();
  Local<Function> callback = Local<Function>::Cast(info[1]);
  AudioIn* obj = Nan::ObjectWrap::Unwrap<AudioIn>(info.Holder());

  AsyncQueueWorker(new InWorker(obj->getContext(), new Nan::Callback(callback)));
  info.GetReturnValue().SetUndefined();
}

NAN_METHOD(AudioIn::Quit) {
  if (info.Length() != 1)
    return Nan::ThrowError("AudioIn Quit expects 1 argument");
  if (!info[0]->IsFunction())
    return Nan::ThrowError("AudioIn Quit requires a valid callback as the parameter");

  Local<Function> callback = Local<Function>::Cast(info[0]);
  AudioIn* obj = Nan::ObjectWrap::Unwrap<AudioIn>(info.Holder());

  AsyncQueueWorker(new QuitInWorker(obj->getContext(), new Nan::Callback(callback)));
  info.GetReturnValue().SetUndefined();
}

NAN_MODULE_INIT(AudioIn::Init) {
  Local<FunctionTemplate> tpl = Nan::New<FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("AudioIn").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  SetPrototypeMethod(tpl, "start", Start);
  SetPrototypeMethod(tpl, "read", Read);
  SetPrototypeMethod(tpl, "quit", Quit);

  constructor().Reset(Nan::GetFunction(tpl).ToLocalChecked());
  Nan::Set(target, Nan::New("AudioIn").ToLocalChecked(),
    Nan::GetFunction(tpl).ToLocalChecked());
}

} // namespace streampunk
