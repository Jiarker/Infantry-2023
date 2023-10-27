#ifndef MODULE_REGISTER_HPP_
#define MODULE_REGISTER_HPP_

#include "CvPreProcessing.hpp"
#include "OVInterpreter.hpp"
#include "RM-V7-PostProcesser.hpp"
#include "RM-V5-PostProcesser.hpp"

void ModuleRegiste(void);

class ModuleRegister
{
public:
    ModuleRegister() {}

    void operator() (void) const
    {
        // per processer registe
        REGISTER_PREPROCESSER(CvStandardAffinePreProcesser, "cv-standard-affine-prep");

        // infer registe
        REGISTER_INTERPRETER(OVInterpreter, "openvino-interpreter");

        // post processer registe
        REGISTER_POSTPROCESSER(RmV5PostProcesser, "rm-v5-post-processer");
        REGISTER_POSTPROCESSER(RmV7PostProcesser, "rm-v7-post-processer");
    }

};

#endif