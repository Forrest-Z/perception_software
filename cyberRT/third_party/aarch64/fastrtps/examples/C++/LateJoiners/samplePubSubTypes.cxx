// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file samplePubSubTypes.cpp
 * This header file contains the implementation of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "samplePubSubTypes.h"

samplePubSubType::samplePubSubType() {
    setName("sample");
    m_typeSize = (uint32_t)sample::getMaxCdrSerializedSize() + 4 /*encapsulation*/;
    m_isGetKeyDefined = sample::isKeyDefined();
    m_keyBuffer = (unsigned char*)malloc(sample::getKeyMaxCdrSerializedSize()>16 ? sample::getKeyMaxCdrSerializedSize() : 16);
}

samplePubSubType::~samplePubSubType() {
    if(m_keyBuffer!=nullptr)
        free(m_keyBuffer);
}

bool samplePubSubType::serialize(void *data, SerializedPayload_t *payload) {
    sample *p_type = (sample*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*) payload->data, payload->max_size); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR);
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    // Serialize encapsulation
    ser.serialize_encapsulation();

    try
    {
        p_type->serialize(ser); // Serialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    payload->length = (uint32_t)ser.getSerializedDataLength(); 	//Get the serialized length
    return true;
}

bool samplePubSubType::deserialize(SerializedPayload_t* payload, void* data) {
    sample* p_type = (sample*) data; 	//Convert DATA to pointer of your type
    eprosima::fastcdr::FastBuffer fastbuffer((char*)payload->data, payload->length); 	// Object that manages the raw buffer.
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that deserializes the data.
    // Deserialize encapsulation.
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;

    try
    {
        p_type->deserialize(deser); //Deserialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    return true;
}

std::function<uint32_t()> samplePubSubType::getSerializedSizeProvider(void* data) {
    return [data]() -> uint32_t
    {
        return (uint32_t)type::getCdrSerializedSize(*static_cast<sample*>(data)) + 4 /*encapsulation*/;
    };
}

void* samplePubSubType::createData() {
    return (void*)new sample();
}

void samplePubSubType::deleteData(void* data) {
    delete((sample*)data);
}

bool samplePubSubType::getKey(void *data, InstanceHandle_t* handle) {
    if(!m_isGetKeyDefined)
        return false;
    sample* p_type = (sample*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*)m_keyBuffer,sample::getKeyMaxCdrSerializedSize()); 	// Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS); 	// Object that serializes the data.
    p_type->serializeKey(ser);
    if(sample::getKeyMaxCdrSerializedSize()>16)	{
        m_md5.init();
        m_md5.update(m_keyBuffer,(unsigned int)ser.getSerializedDataLength());
        m_md5.finalize();
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_md5.digest[i];
        }
    }
    else    {
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_keyBuffer[i];
        }
    }
    return true;
}
