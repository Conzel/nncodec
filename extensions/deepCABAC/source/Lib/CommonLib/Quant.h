/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The NNCodec Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#pragma once

#include <array>
#include "../CommonLib/TypeDef.h"
#include "../CommonLib/Scan.h"
#include "../CommonLib/QuantisationInterfaces.h"
#include "../EncLib/CABACEncoder.h"
#include "../EncLib/BinEncoder.h"

uint32_t quantize(float32_t *weights, int32_t *level, const float32_t qstep, const int32_t stride, const int32_t numTotal, const DistType distType, const double lambdaScale, const uint8_t dq_flag, const uint32_t maxNumNoRem, const int32_t scan_order);
void deQuantize(float32_t *weights, int32_t *level, const float32_t qstep, const uint32_t numWeights, const int32_t stride, const int32_t scan_order);
class CabacRate : protected TCABACEncoder<BinEst>
{
public:
  struct pars
  {
    // int layerwidth;
    uint32_t maxNumNoRem;
  };

public:
  // the constructor and the functions must have exactly this form
  CabacRate(int32_t stateId, const pars &p);
  void copyCtx(const CabacRate *other);
  void updateCtx(int32_t level);
  double operator()(int32_t level);

private:
  const int32_t m_stateId;
};

class RateEstimation
{
public:
  RateEstimation(uint32_t cabac_unary_length_minus1 = 10) : rateEst(0, {cabac_unary_length_minus1}) {}
  ~RateEstimation() {}

  float32_t estimate(int32_t quantisation_index)
  {
    return rateEst(quantisation_index) / this->scale;
  }
  void update(int32_t quantisation_index)
  {
    rateEst.updateCtx(quantisation_index);
  }

private:
  CabacRate rateEst;
  double scale = double(1 << 15);
};
