// Guitar Amp Sim - distingNT - v15
// Mono in, stereo out with independent IR per channel
// Input Gain + Drive (4 types) + IR L/R + Output Level
// GUID: GtAm

#include <distingnt/api.h>
#include <distingnt/wav.h>
#include <cstddef>
#include <new>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int   kMaxIrLen = 128;
static const char* kIrFolder = "CabIR";

enum {
    kParamInput       = 0,
    kParamOutputL     = 1,
    kParamOutputLMode = 2,
    kParamOutputR     = 3,
    kParamOutputRMode = 4,
    kParamInputGain   = 5,
    kParamDriveType   = 6,
    kParamDrive       = 7,
    kParamOutputLevel = 8,
    kParamIrSelectL   = 9,
    kParamIrSelectR   = 10,
};

// DC blocker — simple difference equation, proven stable
struct DCBlock {
    float x1, y1;
    void init() { x1=y1=0.f; }
    // R close to 1 = very low cutoff (~5Hz removes DC only)
    inline float process(float x, float R) {
        float y = x - x1 + R*y1;
        x1=x; y1=y; return y;
    }
};

struct _IrChannel {
    float irBuffer [kMaxIrLen];
    float delayLine[kMaxIrLen];
};

struct _GAS_DRAM {
    _IrChannel L;
    _IrChannel R;
};

struct _Alg : public _NT_algorithm {
    _GAS_DRAM*     dram;
    _NT_wavRequest wavRequestL;
    _NT_wavRequest wavRequestR;
    int   irLenL,    irLenR;
    int   delayPosL, delayPosR;
    bool  irLoadedL,     irLoadedR;
    bool  awaitingLoadL, awaitingLoadR;
    bool  cardMounted,   folderScanned;
    int   irFolderIndex, irSampleCount;
    float irNormGainL,   irNormGainR;
    DCBlock dcBlock;        // removes DC from Hi-Z pickup
    _NT_parameter params[11];
};

static const char* kDriveTypeNames[] = {
    "Soft Clip", "Hard Clip", "Fuzz", "Tape", NULL
};

static const _NT_parameter kDefaultParams[] = {
    NT_PARAMETER_AUDIO_INPUT("Input", 1, 1)                    // 0
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output L", 1, 13)     // 1+2
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output R", 1, 14)     // 3+4
    { "Input Gain", -20, 40,  0, kNT_unitDb,      0, NULL },   // 5
    { "Drive Type",   0,  3,  0, kNT_unitEnum,    0, kDriveTypeNames }, // 6
    { "Drive",        0,100,  0, kNT_unitPercent,  0, NULL },   // 7
    { "Out Level",  -20, 20,  0, kNT_unitDb,       0, NULL },   // 8
    { "IR Left",      0,  0,  0, kNT_unitNone,     0, NULL },   // 9
    { "IR Right",     0,  0,  0, kNT_unitNone,     0, NULL },   // 10
};

static const uint8_t kPageIO[]  = { kParamInput,
                                    kParamOutputL, kParamOutputLMode,
                                    kParamOutputR, kParamOutputRMode };
static const uint8_t kPageAmp[] = { kParamInputGain, kParamDriveType,
                                    kParamDrive, kParamOutputLevel };
static const uint8_t kPageCab[] = { kParamIrSelectL, kParamIrSelectR };

static const _NT_parameterPage kPages[] = {
    { "I/O", 5, 0, {0,0}, kPageIO  },
    { "Amp", 4, 0, {0,0}, kPageAmp },
    { "Cab", 2, 0, {0,0}, kPageCab },
};
static const _NT_parameterPages kParamPages = { 3, kPages };

static float calcNorm(const float* buf, int len) {
    float peak = 0.f;
    for (int i = 0; i < len; i++) {
        float a = fabsf(buf[i]);
        if (a > peak) peak = a;
    }
    float n = (peak > 1e-6f) ? 1.f/peak : 1.f;
    return (n > 10.f) ? 10.f : n;
}

static void irLoadCallbackL(void* data, bool ok) {
    _Alg* p = (_Alg*)data;
    p->awaitingLoadL = false;
    if (!ok || p->irLenL == 0) { p->irLoadedL = false; return; }
    p->irNormGainL = calcNorm(p->dram->L.irBuffer, p->irLenL);
    memset(p->dram->L.delayLine, 0, kMaxIrLen * sizeof(float));
    p->delayPosL = 0;
    p->irLoadedL = true;
}

static void irLoadCallbackR(void* data, bool ok) {
    _Alg* p = (_Alg*)data;
    p->awaitingLoadR = false;
    if (!ok || p->irLenR == 0) { p->irLoadedR = false; return; }
    p->irNormGainR = calcNorm(p->dram->R.irBuffer, p->irLenR);
    memset(p->dram->R.delayLine, 0, kMaxIrLen * sizeof(float));
    p->delayPosR = 0;
    p->irLoadedR = true;
}

static void loadIr(_Alg* p, int sel, bool isRight) {
    if (p->irFolderIndex < 0 || p->irSampleCount <= 0) return;
    if (sel < 0 || sel >= p->irSampleCount) return;
    if (isRight && p->awaitingLoadR) return;
    if (!isRight && p->awaitingLoadL) return;
    _NT_wavInfo info;
    NT_getSampleFileInfo((uint32_t)p->irFolderIndex, (uint32_t)sel, info);
    int frames = (int)info.numFrames;
    if (frames > kMaxIrLen) frames = kMaxIrLen;
    if (frames <= 0) return;
    _NT_wavRequest* req = isRight ? &p->wavRequestR : &p->wavRequestL;
    req->folder       = (uint32_t)p->irFolderIndex;
    req->sample       = (uint32_t)sel;
    req->dst          = isRight ? p->dram->R.irBuffer : p->dram->L.irBuffer;
    req->numFrames    = (uint32_t)frames;
    req->startOffset  = 0;
    req->channels     = kNT_WavMono;
    req->bits         = kNT_WavBits32;
    req->progress     = kNT_WavNoProgress;
    req->callback     = isRight ? irLoadCallbackR : irLoadCallbackL;
    req->callbackData = p;
    if (isRight) {
        p->irLenR = frames; p->irLoadedR = false;
        p->awaitingLoadR = NT_readSampleFrames(*req);
    } else {
        p->irLenL = frames; p->irLoadedL = false;
        p->awaitingLoadL = NT_readSampleFrames(*req);
    }
}

static void scanFolder(_Alg* p, int algIdx) {
    p->folderScanned = true;
    uint32_t n = NT_getNumSampleFolders();
    for (uint32_t i = 0; i < n; i++) {
        _NT_wavFolderInfo fi;
        NT_getSampleFolderInfo(i, fi);
        if (fi.name && strcmp(fi.name, kIrFolder) == 0) {
            p->irFolderIndex = (int)i;
            p->irSampleCount = (int)fi.numSampleFiles;
            if (p->irSampleCount > 0 && algIdx >= 0) {
                p->params[kParamIrSelectL].max  = p->irSampleCount - 1;
                p->params[kParamIrSelectL].unit = kNT_unitHasStrings;
                p->params[kParamIrSelectR].max  = p->irSampleCount - 1;
                p->params[kParamIrSelectR].unit = kNT_unitHasStrings;
                NT_updateParameterDefinition((uint32_t)algIdx, kParamIrSelectL);
                NT_updateParameterDefinition((uint32_t)algIdx, kParamIrSelectR);
            }
            loadIr(p, p->v[kParamIrSelectL], false);
            loadIr(p, p->v[kParamIrSelectR], true);
            return;
        }
    }
}

static void calculateRequirements(_NT_algorithmRequirements& req, const int32_t*) {
    req.numParameters = ARRAY_SIZE(kDefaultParams);
    req.sram = sizeof(_Alg);
    req.dram = sizeof(_GAS_DRAM);
    req.dtc = req.itc = 0;
}

static _NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                                const _NT_algorithmRequirements&, const int32_t*) {
    _Alg* alg = new (ptrs.sram) _Alg();
    alg->dram = (_GAS_DRAM*)ptrs.dram;
    memset(alg->dram, 0, sizeof(_GAS_DRAM));
    memcpy(alg->params, kDefaultParams, sizeof(kDefaultParams));
    alg->parameters     = alg->params;
    alg->parameterPages = &kParamPages;
    alg->irLenL = alg->irLenR = 0;
    alg->delayPosL = alg->delayPosR = 0;
    alg->irLoadedL = alg->irLoadedR = false;
    alg->awaitingLoadL = alg->awaitingLoadR = false;
    alg->cardMounted = alg->folderScanned = false;
    alg->irFolderIndex = -1;
    alg->irSampleCount = 0;
    alg->irNormGainL = alg->irNormGainR = 1.f;
    alg->dcBlock.init();
    return alg;
}

static void parameterChanged(_NT_algorithm* self, int p) {
    _Alg* pThis = (_Alg*)self;
    if (p == kParamIrSelectL) loadIr(pThis, pThis->v[kParamIrSelectL], false);
    if (p == kParamIrSelectR) loadIr(pThis, pThis->v[kParamIrSelectR], true);
}

static int parameterString(_NT_algorithm* self, int p, int v, char* buff) {
    _Alg* pThis = (_Alg*)self;
    if ((p == kParamIrSelectL || p == kParamIrSelectR) &&
        pThis->irFolderIndex >= 0 && v >= 0 && v < pThis->irSampleCount) {
        _NT_wavInfo info;
        NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex, (uint32_t)v, info);
        if (info.name) {
            strncpy(buff, info.name, kNT_parameterStringSize-1);
            buff[kNT_parameterStringSize-1] = 0;
            return (int)strlen(buff);
        }
    }
    return 0;
}

static inline float saturate(float s, int type, float drive) {
    if (drive <= 0.f) return s;
    float pregain = 1.f + drive * 7.f;
    float x = s * pregain;
    float sat;
    switch (type) {
        case 1: {
            float thresh = 1.f - drive * 0.6f;
            sat = (x > thresh) ? thresh : (x < -thresh) ? -thresh : x;
        } break;
        case 2:
            sat = tanhf(x * (1.f + fabsf(x) * 0.5f) * 1.5f);
            break;
        case 3: {
            float xc = x * 0.8f;
            if (xc >  1.f) xc =  1.f;
            if (xc < -1.f) xc = -1.f;
            sat = xc - (xc*xc*xc)/3.f + xc*xc*0.1f;
        } break;
        default:
            sat = tanhf(x);
            break;
    }
    return sat / pregain;
}

static void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _Alg* pThis = (_Alg*)self;
    int numFrames = numFramesBy4 * 4;

    int inBus   = pThis->v[kParamInput]   - 1;
    int outBusL = pThis->v[kParamOutputL] - 1;
    int outBusR = pThis->v[kParamOutputR] - 1;
    if (inBus < 0 || outBusL < 0 || outBusR < 0) return;

    bool mounted = NT_isSdCardMounted();
    if (mounted != pThis->cardMounted) {
        pThis->cardMounted   = mounted;
        pThis->folderScanned = false;
    }
    if (mounted && !pThis->folderScanned)
        scanFolder(pThis, NT_algorithmIndex(self));

    const float* in   = busFrames + inBus   * numFrames;
    float*       outL = busFrames + outBusL * numFrames;
    float*       outR = busFrames + outBusR * numFrames;
    bool replaceL = (pThis->v[kParamOutputLMode] != 0);
    bool replaceR = (pThis->v[kParamOutputRMode] != 0);

    float inGain  = powf(10.f, pThis->v[kParamInputGain]  / 20.f);
    float dcR     = 1.f - 2.f*(float)M_PI*40.f/(float)NT_globals.sampleRate;
    float outGain = powf(10.f, pThis->v[kParamOutputLevel] / 20.f);
    float drive   = pThis->v[kParamDrive] / 100.f;
    int   drvType = pThis->v[kParamDriveType];

    float* irL   = pThis->dram->L.irBuffer;
    float* dlL   = pThis->dram->L.delayLine;
    float* irR   = pThis->dram->R.irBuffer;
    float* dlR   = pThis->dram->R.delayLine;
    int    lenL  = pThis->irLenL,      lenR  = pThis->irLenR;
    float  normL = pThis->irNormGainL, normR = pThis->irNormGainR;
    int    dposL = pThis->delayPosL,   dposR = pThis->delayPosR;
    bool   hasIrL = pThis->irLoadedL && lenL > 0;
    bool   hasIrR = pThis->irLoadedR && lenR > 0;

    float tmp[128];
    int done = 0;
    while (done < numFrames) {
        int batch = numFrames - done;
        if (batch > 128) batch = 128;
        for (int i = 0; i < batch; i++) tmp[i] = in[done+i];

        for (int i = 0; i < batch; i++) {
            float s = pThis->dcBlock.process(tmp[i] * inGain, dcR);
            s = saturate(s, drvType, drive);

            float convL = s;
            if (hasIrL) {
                dlL[dposL] = s;
                convL = 0.f;
                int rpos = dposL;
                for (int k = 0; k < lenL; k++) {
                    convL += irL[k] * dlL[rpos];
                    if (--rpos < 0) rpos = lenL-1;
                }
                convL *= normL;
                if (++dposL >= lenL) dposL = 0;
            }

            float convR = s;
            if (hasIrR) {
                dlR[dposR] = s;
                convR = 0.f;
                int rpos = dposR;
                for (int k = 0; k < lenR; k++) {
                    convR += irR[k] * dlR[rpos];
                    if (--rpos < 0) rpos = lenR-1;
                }
                convR *= normR;
                if (++dposR >= lenR) dposR = 0;
            }

            outL[done+i] = replaceL ? convL*outGain : outL[done+i] + convL*outGain;
            outR[done+i] = replaceR ? convR*outGain : outR[done+i] + convR*outGain;
        }
        done += batch;
    }
    pThis->delayPosL = dposL;
    pThis->delayPosR = dposR;
}

// Draw a horizontal bar meter, left-aligned at (x,y), width w, height h
// value 0..100, filled proportionally
static void drawBar(int x, int y, int w, int h, int value, int col) {
    NT_drawShapeI(kNT_box, x, y, x+w, y+h, 6);  // outline dim
    int fill = (value * (w-2)) / 100;
    if (fill > 0)
        NT_drawShapeI(kNT_rectangle, x+1, y+1, x+1+fill, y+h-1, col);
}

static bool draw(_NT_algorithm* self) {
    _Alg* pThis = (_Alg*)self;

    // Title bar
    NT_drawShapeI(kNT_rectangle, 0, 0, 255, 10, 10);
    NT_drawText(128, 8, "Guitar Amp Sim", 15, kNT_textCentre, kNT_textNormal);

    // --- Left column: Amp section ---
    NT_drawText(2, 19, "IN", 8, kNT_textLeft, kNT_textTiny);
    NT_drawText(2, 27, "DRV", 8, kNT_textLeft, kNT_textTiny);
    NT_drawText(2, 35, "OUT", 8, kNT_textLeft, kNT_textTiny);

    // Input gain bar: -20..+40 range, map to 0..100
    int inVal = (int)((pThis->v[kParamInputGain] + 20) * 100 / 60);
    if (inVal < 0) inVal = 0; if (inVal > 100) inVal = 100;
    drawBar(18, 13, 80, 7, inVal, 12);

    // Drive bar: 0..100
    drawBar(18, 21, 80, 7, pThis->v[kParamDrive], 11);

    // Out level bar: -20..+20 range
    int outVal = (int)((pThis->v[kParamOutputLevel] + 20) * 100 / 40);
    if (outVal < 0) outVal = 0; if (outVal > 100) outVal = 100;
    drawBar(18, 29, 80, 7, outVal, 12);

    // Drive type label
    static const char* driveNames[] = { "Soft", "Hard", "Fuzz", "Tape" };
    int dt = pThis->v[kParamDriveType];
    if (dt >= 0 && dt <= 3)
        NT_drawText(18, 43, driveNames[dt], 13, kNT_textLeft, kNT_textTiny);

    // --- Divider ---
    NT_drawShapeI(kNT_line, 103, 11, 103, 63, 6);

    // --- Right column: IR / status ---
    NT_drawText(107, 19, "CAB", 8, kNT_textLeft, kNT_textTiny);

    if (!pThis->cardMounted) {
        NT_drawText(107, 30, "No SD card", 7, kNT_textLeft, kNT_textTiny);
    } else if (pThis->irFolderIndex < 0) {
        NT_drawText(107, 30, "No CabIR", 7, kNT_textLeft, kNT_textTiny);
        NT_drawText(107, 38, "folder", 7, kNT_textLeft, kNT_textTiny);
    } else if (pThis->awaitingLoadL || pThis->awaitingLoadR) {
        NT_drawText(107, 30, "Loading...", 10, kNT_textLeft, kNT_textTiny);
    } else {
        // Show L and R IR names (truncated to fit)
        auto showIr = [&](int param, int y, const char* prefix) {
            NT_drawText(107, y, prefix, 8, kNT_textLeft, kNT_textTiny);
            int sel = pThis->v[param];
            if (sel >= 0 && sel < pThis->irSampleCount) {
                _NT_wavInfo info;
                NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex,
                                     (uint32_t)sel, info);
                if (info.name)
                    NT_drawText(119, y, info.name, 12, kNT_textLeft, kNT_textTiny);
            }
        };
        showIr(kParamIrSelectL, 27, "L:");
        showIr(kParamIrSelectR, 35, "R:");

        // IR loaded indicator dots
        NT_drawShapeI(kNT_rectangle, 107, 43,
                      pThis->irLoadedL ? 113 : 108, 48,
                      pThis->irLoadedL ? 12 : 5);
        NT_drawShapeI(kNT_rectangle, 116, 43,
                      pThis->irLoadedR ? 122 : 117, 48,
                      pThis->irLoadedR ? 12 : 5);
        NT_drawText(107, 56, pThis->irLoadedL ? "L:OK" : "L:--", 8, kNT_textLeft, kNT_textTiny);
        NT_drawText(125, 56, pThis->irLoadedR ? "R:OK" : "R:--", 8, kNT_textLeft, kNT_textTiny);
    }

    return true;  // true = we drew everything, don't show standard param line
}

static const _NT_factory factory = {
    .guid = NT_MULTICHAR('F','L','M','P'),
    .name = "Guitar Amp Sim",
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = parameterChanged,
    .step = step,
    .draw = draw,
    .tags = kNT_tagEffect,
    .parameterString = parameterString,
};

extern "C"
uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:      return kNT_apiVersionCurrent;
        case kNT_selector_numFactories: return 1;
        case kNT_selector_factoryInfo:  return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}
