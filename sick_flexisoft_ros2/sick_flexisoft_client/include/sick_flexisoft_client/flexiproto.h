#ifndef H_FLEXI_PROTO
#define H_FLEXI_PROTO

#include <string.h>
#include <stdint.h>
#include <vector>

namespace flexi {
    
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
    
struct FlexiWord{
    uint8_t hi;
    uint8_t lo;
    FlexiWord& operator=( const uint16_t & v ) {
        hi = v >> 8; lo = v & 0xFF;
        return *this;
    }    
    operator uint16_t() const
    {
        return (hi << 8) | lo;
    }
};

struct FlexiHeader{
    FlexiWord command;
     enum CommandType {
        UNDEFINED = 0x0000, READ_DATA = 0x00F1, READ_DATA_REPLY = 0x001F, UPDATE_CONTROL = 0x00E1, UPDATE_CONTROL_REPLY = 0x001E, UPDATE_DATA = 0x002E, WRITE_DATA = 0x00F2 , WRITE_DATA_REPLY = 0x002F        
    };
    CommandType get_reply_type() const{
        switch(command){
            case READ_DATA:
                return READ_DATA_REPLY;
            case UPDATE_CONTROL:
                return UPDATE_CONTROL_REPLY;
            case WRITE_DATA:
                return WRITE_DATA_REPLY;
            default:
                return UNDEFINED;
        }
    }
    CommandType get_type() const { return CommandType(command & 0x7FFFF);  }
    bool is_error() const { return command.hi & 0x80; }
    void set_type(CommandType c) { command = c & 0x7FFFF;  }
    void set_error(bool e) { 
        if(e)  command.hi |= 0x80;
        else command.hi &= 0x7F;
    }
};

struct FlexiCommand{
    FlexiHeader header;
};

template <uint8_t N,uint8_t M> struct FlexiFieldData: public FlexiCommand{
    static const uint16_t max_fields = N;
    static const uint16_t max_size = 2 + N*2 + M;
    static const uint16_t max_data_length = M;
    FlexiWord lengths[N];
    uint8_t data[M];
    
    std::vector<uint8_t> get_field(uint8_t field) const{
        if( field >= max_fields) return std::vector<uint8_t>();
        
        uint16_t offset = 0;
        for(uint8_t i=0; i < field; ++i) offset += lengths[i];
        
        return std::vector<uint8_t>( &data[offset], &data[offset] + lengths[field] );
    }
    uint16_t length() const{
        uint16_t size = 2 + max_fields*2;
        for(uint8_t i=0; i < max_fields; ++i) size += lengths[i];
        if(size > max_size) size = max_size;
        return size;
    }
    uint16_t missing_bytes(uint16_t length) const{
        uint16_t size = 2 + max_fields*2;
        if(length < size) return size - length;
        for(uint8_t i=0; i < max_fields; ++i) size += lengths[i];
        if(size > max_size) size = max_size;
        if(length < size) return size - length;
        return 0;
    }
};

struct FlexiInputData: public FlexiFieldData<4,50+32+60+60>{    
};
struct FlexiOutputData: public FlexiFieldData<5,5*10>{    
};

struct FlexiStatus: public FlexiCommand{
    FlexiWord status;
};

struct FlexiPoll: public FlexiCommand{
    FlexiWord fields[4];
};

struct FlexiUpdateControl: public FlexiPoll{
    FlexiWord heartbeat;
};

#pragma pack(pop)  /* push current alignment to stack */

struct FlexiMsg{
    uint16_t length;
    union{
        FlexiHeader header;
        FlexiInputData input;
        FlexiOutputData output;
        FlexiPoll poll;
        FlexiUpdateControl control;
        FlexiStatus status;
    } payload;

    bool is_error() const {
        if(length >= 2) return payload.header.is_error();
        else return true;
    }

    FlexiHeader::CommandType get_type() const {
        if(length >= 2) return payload.header.get_type();
        else return FlexiHeader::UNDEFINED;
    }

    FlexiHeader::CommandType get_reply_type() const {
        if(length >= 2) return payload.header.get_reply_type();
        else return FlexiHeader::UNDEFINED;
    }
    
    FlexiMsg(): length(0) { }
    
    uint16_t missing_bytes() const {
        uint16_t min_length;
        if(length < 2) return 2;
        switch(payload.header.get_type()){
            case FlexiHeader::READ_DATA:
                min_length = sizeof(FlexiPoll);
                break;
            case FlexiHeader::READ_DATA_REPLY:
            case FlexiHeader::UPDATE_DATA:
                min_length = length + payload.input.missing_bytes(length);
                break;
            case FlexiHeader::UPDATE_CONTROL:
                min_length = sizeof(FlexiUpdateControl);
                break;
            case FlexiHeader::UPDATE_CONTROL_REPLY:
                min_length = sizeof(FlexiHeader);
                break;
            case FlexiHeader::WRITE_DATA:
                min_length = length + payload.output.missing_bytes(length);
                break;
            case FlexiHeader::WRITE_DATA_REPLY:
                min_length = sizeof(FlexiStatus);
                break;
            default:
                min_length = 2;
        }
        if(length < min_length) return min_length - length;
        return 0;
    }
    
    void set_control(bool f1 , bool f2, bool f3, bool f4, uint16_t heartbeat){
        set_poll(f1,f2,f3,f4);
        payload.header.command = FlexiHeader::UPDATE_CONTROL;
        payload.control.heartbeat = heartbeat;
        length = sizeof(FlexiUpdateControl);
    }
    void set_poll(bool f1 , bool f2, bool f3, bool f4){
        payload.header.command = FlexiHeader::READ_DATA;
        payload.control.fields[0] = f1 ? 1 : 0;
        payload.control.fields[1] = f2 ? 1 : 0;
        payload.control.fields[2] = f3 ? 1 : 0;
        payload.control.fields[3] = f4 ? 1 : 0;
        length = sizeof(FlexiPoll);
    }
    void set_output(const std::vector<uint8_t> fields[FlexiOutputData::max_fields]){
        payload.header.command = FlexiHeader::WRITE_DATA;
        
        uint8_t *data = payload.output.data;
        
        for(uint8_t i = 0; i < FlexiOutputData::max_fields; ++i){
            if(fields[i].size()){
                payload.output.lengths[i] = 10;
                memcpy(data, fields[i].data(), fields[i].size() < 10 ?  fields[i].size() : 10);
                data += 10;
            }else payload.output.lengths[i] = 0;
        }
        
        length = payload.output.length();

    }
};

};

#endif