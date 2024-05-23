#include "daisysp.h"
#include "teil2_control.h"


#include "../../CloudSeed/Default.h"
#include "../../CloudSeed/ReverbController.h"
#include "../../CloudSeed/FastSin.h"
#include "../../CloudSeed/AudioLib/ValueTables.h"
#include "../../CloudSeed/AudioLib/MathDefs.h"

using namespace daisy;
using namespace daisysp;
static Teil2Control teil2_control;
static WhiteNoise wnoise;


::daisy::Parameter hpFilter_Parameter;

static float drylevel, send, noisegain;

CloudSeed::ReverbController* reverb = 0;

#define CUSTOM_POOL_SIZE (48*1024*1024)

DSY_SDRAM_BSS char custom_pool[CUSTOM_POOL_SIZE];

size_t pool_index = 0;
int allocation_count = 0;

void* custom_pool_allocate(size_t size)
{
        if (pool_index + size >= CUSTOM_POOL_SIZE)
        {
                return 0;
        }
        void* ptr = &custom_pool[pool_index];
        pool_index += size;
        return ptr;
}


float ctrlVal[8];
float prevCtrlVal[8];



static void VerbCallback(float **in, float **out, size_t size)
{
    send = 1.0;
    float dryL, dryR, wetL, wetR, sendL, sendR;
    float noisesig;
     // read some controls
    
    //teil2_control.UpdateAnalogControls();       // also done later by process
    teil2_control.DebounceControls();     // for Buttons

    
    
    for (int i = 0; i < 8; i++)
    {
        //Get the four control values
        ctrlVal[i] = teil2_control.knobs[i].Process();
        if (ctrlVal[i]<0.01)
           ctrlVal[i] = 0;
        if (ctrlVal[i]>0.97)
           ctrlVal[i]=1;
    }
    


    //drylevel = ctrlVal[0];
    //prevCtrlVal[0] = ctrlVal[0];

    
    
    
    float delta = 0.01;
   

    
    if ((prevCtrlVal[0] < (ctrlVal[0]-delta)) || (prevCtrlVal[0] > (ctrlVal[0]+delta)))
    {
      //reverb->SetParameter(::Parameter::LineDecay, ctrlVal[0]);
      prevCtrlVal[0] = ctrlVal[0];
    }
    if ((prevCtrlVal[1] < (ctrlVal[1]-delta)) || (prevCtrlVal[1] > (ctrlVal[1]+delta)))
    {
      //reverb->SetParameter(::Parameter::MainOut, ctrlVal[1]);
      prevCtrlVal[1] = ctrlVal[1];
    }
    if ((prevCtrlVal[2] < (ctrlVal[2]-delta)) || (prevCtrlVal[2] > (ctrlVal[2]+delta)))
    {
      reverb->SetParameter(::Parameter::LineDecay, ctrlVal[2]);
      prevCtrlVal[2] = ctrlVal[2];
    }
    if ((prevCtrlVal[3] < (ctrlVal[3]-delta)) || (prevCtrlVal[3] > (ctrlVal[3]+delta)))
    {
      reverb->SetParameter(::Parameter::LateDiffusionFeedback, ctrlVal[3]);

      prevCtrlVal[3] = ctrlVal[3];
    }
    if ((prevCtrlVal[4] < (ctrlVal[4]-delta)) || (prevCtrlVal[4] > (ctrlVal[4]+delta)))
    {
      //reverb->SetParameter(::Parameter::LateDiffusionFeedback, ctrlVal[3]);
      noisegain = ctrlVal[4];
      prevCtrlVal[4] = ctrlVal[4];
    }
    if ((prevCtrlVal[5] < (ctrlVal[5]-delta)) || (prevCtrlVal[5] > (ctrlVal[5]+delta)))
    {
      //reverb->SetParameter(::Parameter::LateDiffusionFeedback, ctrlVal[3]);
      prevCtrlVal[5] = ctrlVal[5];
    }
    if ((prevCtrlVal[6] < (ctrlVal[6]-delta)) || (prevCtrlVal[6] > (ctrlVal[6]+delta)))
    {
      //reverb->SetParameter(::Parameter::LateDiffusionFeedback, ctrlVal[3]);
      prevCtrlVal[6] = ctrlVal[6];
    }
    if ((prevCtrlVal[7] < (ctrlVal[7]-delta)) || (prevCtrlVal[7] > (ctrlVal[7]+delta)))
    {
      //reverb->SetParameter(::Parameter::LateDiffusionFeedback, ctrlVal[3]);
      prevCtrlVal[7] = ctrlVal[7];
    }


    drylevel = 0.8;
    reverb->SetParameter(::Parameter::MainOut, 0.9);
    //reverb->SetParameter(::Parameter::LineDecay, 0.5);
    //reverb->SetParameter(::Parameter::LateDiffusionFeedback, 0.4);


    

    for (size_t i = 0; i < size; i++)
    {
        noisesig = wnoise.Process();

        // Read Inputs (only stereo in are used)
        dryL = in[0][i]*drylevel;
        dryR = in[1][i]*drylevel;

        // Send Signal to Reverb
        //sendL = ( dryL * send + (noisesig*noisegain) )/2.0;
        //sendR = ( dryR * send + (noisesig*noisegain) )/2.0;
        sendL =  dryL * send 
        sendR =  dryR * send 

        //verb.Process(sendL, sendR, &wetL, &wetR);
        float ins[2]={sendL,sendR};
        float outs[2]={sendL,sendR};
        reverb->Process( ins, outs, 1);
        
        wetL=outs[0];
        wetR=outs[1];   


        out[0][i] = ( outs[0] + (noisesig*noisegain) )/2.0;
        out[1][i] = ( outs[1] + (noisesig*noisegain) )/2.0;

    }
}





int main(void)
{
    float samplerate;
    teil2_control.Init();
    samplerate = teil2_control.AudioSampleRate();
    AudioLib::ValueTables::Init();
    CloudSeed::FastSin::Init();
    reverb = new CloudSeed::ReverbController(samplerate);
    reverb->ClearBuffers();
    reverb->initFactoryChorus();
    wnoise.Init();


    hpFilter_Parameter.Init(teil2_control.knobs[7], 20, 18000, ::daisy::Parameter::LOGARITHMIC);


    teil2_control.StartAdc();
    teil2_control.StartAudio(VerbCallback);

    while(1) 
    {
      
      teil2_control.DelayMs(100);

    }

}

