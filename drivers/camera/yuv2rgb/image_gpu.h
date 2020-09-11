/*!
* \brief This file defines image gpu header
* \attention Copyright Geely Car Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#pragma once

#include <cuda_runtime.h>
#include <cuda.h>

#include <iostream>

void CudaYUYVToRGB(const int32_t& cols, 
                    const int32_t& rows, 
                    uint8_t* cuda_yuyv, 
                    uint8_t* cuda_rgb, 
                    uint8_t* yuyv, 
                    uint8_t* rgbb_image);
