// RHGenericSPI.cpp
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// Contributed by Joanna Rutkowska
// $Id: RHGenericSPI.cpp,v 1.2 2014/04/12 05:26:05 mikem Exp $

#ifdef PIO_BUILD_ENV
#include <RadioHead\RHGenericSPI.h>
#else
#include <../Common/Submodules/RadioHead/RHGenericSPI.h>
#endif

RHGenericSPI::RHGenericSPI(Frequency frequency, BitOrder bitOrder, DataMode dataMode)
    :
    _frequency(frequency),
    _bitOrder(bitOrder),
    _dataMode(dataMode)
{
}

void RHGenericSPI::setBitOrder(BitOrder bitOrder)
{
    _bitOrder = bitOrder;
}

void RHGenericSPI::setDataMode(DataMode dataMode)
{
    _dataMode = dataMode; 
}

void RHGenericSPI::setFrequency(Frequency frequency)
{
    _frequency = frequency;
}

