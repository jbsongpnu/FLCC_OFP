#include "LCIND.h"

extern const AP_HAL::HAL& hal;

RingBuffer::RingBuffer() {
    head = 0;
    tail = 0;
    buffer[0] = 0;buffer[1]=0;
}

bool RingBuffer::push(uint8_t byte) {
    uint16_t next = (head + 1) % BUF_SIZE;
    if (next == tail) return false; // overflow
    buffer[head] = byte;
    head = next;
    return true;
}

bool RingBuffer::pop_frame(uint8_t* out_frame) {
    uint16_t i = tail;
    while (i != head) {
        if (buffer[i] == 0x02) {
            if (distance(i, head) >= 21) {
                bool full_frame_ready = true;
                for (uint16_t j = 0; j < 21; ++j) {
                    uint16_t index = (i + j) % BUF_SIZE;
                    if (index == head) {  // 아직 쓰여지지 않은 위치에 도달
                        full_frame_ready = false;
                        break;
                    }
                }
                if (full_frame_ready && buffer[(i + 20) % BUF_SIZE] == 0x03) {
                    for (uint16_t j = 0; j < 21; ++j) {
                        out_frame[j] = buffer[(i + j) % BUF_SIZE];
                    }
                    tail = (i + 21) % BUF_SIZE;
                    return true;
                }
            }
        }
        i = (i + 1) % BUF_SIZE;
    }
    return false;
}

uint16_t RingBuffer::distance(uint16_t from, uint16_t to) const {
    return (to + BUF_SIZE - from) % BUF_SIZE;
}

void RingBuffer::reset() {
    head = 0;
    tail = 0;
    buffer[0] = 0;buffer[1]=0;
}

LCIND_class::LCIND_class()
{
    if (_singleton) {
        return;
    }
    _singleton = this;
}

LCIND_class *LCIND_class::_singleton = nullptr;
LCIND_class *LCIND_class::get_singleton()
{
    return _singleton;
}

bool LCIND_class::parse(const uint8_t* data, uint16_t len, uint8_t device) {
    if (len != 21) {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: Invalid length");
        return false;
    }
    if (data[0] != 0x02 || data[20] != 0x03) {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: Invalid start/end markers");
        return false;
    }
    if (device > 3) {
        return false;
    }

    //ID
    if (!isdigit(data[1]) || !isdigit(data[2])) return false;
    id[device] = (data[1] - '0') * 10 + (data[2] - '0');

    // RCWT Check
    if (!(data[3] == 'R' && data[4] == 'C' && data[5] == 'W' && data[6] == 'T')) {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: RCWT mismatch");
        return false;
    }

    // State1
    switch (data[7]) {
        case 'O': 
            state1[device] = Max_Exceeded; 
            break;
        case 'S': 
            state1[device] = Stable; 
            break;
        case 'U': 
            state1[device] = Unstable; 
            break;
        default: 
            return false;
    }

    // State2
    switch (data[8]) {
        case 'N': 
            state2[device] = Real_Weight; 
            break;
        case 'G': 
            state2[device] = Gross_Weight; 
            break;
        default: 
            return false;
    }

    // P check
    if (data[9] != 'P') {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: Missing P flag");
        return false;
    }

    // digit
    if (!isdigit(data[10]) || data[10] > '5') return false;
    int decimal_point = data[10] - '0';

    // sign
    bool negative;
    if (data[11] == '+') {
        negative = false;
    } else if (data[11] == '-') {
        negative = true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: +/- sign error");
        return false;
    }

    // weight value
    char weight_buf[7] = {0};
    for (int i = 0; i < 6; ++i) {
        if (!isdigit(data[12 + i])) return false;
        weight_buf[i] = data[12 + i];
    }
    int raw_weight = std::atoi(weight_buf);
    weight[device] = raw_weight / powf(10.0f, decimal_point); //powf creates 10^n value
    if (negative) {
        weight[device] = -1 * weight[device];
    }

    // unit
    if (data[18] == 'k' && data[19] == 'g') {
        unit[device] = KG;
    } else if (data[18] == 'l' && data[19] == 'b') {
        unit[device] = LB;
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: Invalid unit");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "LCIND error: Invalid unit");
        return false;
    }

    valid[device] = true;
    return true;
}

void LCIND_class::process_stream(RingBuffer& rb, const uint8_t* incoming, uint16_t length, uint8_t device) {
    if (device > 3) return;
    for (uint16_t i = 0; i < length; ++i) {
        rb.push(incoming[i]);
        
        //1st application
        //uint8_t frame[21];
        // while (rb.pop_frame(frame)) {   
        //     parse(frame, sizeof(frame), device);
            // if (parse(frame, sizeof(frame), device)) {
            //     gcs().send_text(MAV_SEVERITY_INFO, "parsing : len %u, device %u", length, device);
            // } else {
            //     gcs().send_text(MAV_SEVERITY_INFO, "parsing failed : len %u, device %u", length, device);
            // }
        // }
    }
    //2nd application
    if(rb.pop_frame(frame)) {
        parse(frame, FRAME_SIZE, device);
    }
}

void LCIND_class::TX_Set_Zero() {
    uint8_t TX_buf[8] = {0};
    TX_buf[0] = 0x02;
    TX_buf[1] = '0';
    TX_buf[2] = '1';//Device ID 1
    TX_buf[3] = 'W';
    TX_buf[4] = 'Z';
    TX_buf[5] = 'E';
    TX_buf[6] = 'R';
    TX_buf[7] = 0x03;
    UART_LC_1->write(TX_buf, 7);

    TX_buf[2] = '2';//Device ID 2
    UART_LC_2->write(TX_buf, 7);

    TX_buf[2] = '3';//Device ID 3
    UART_LC_3->write(TX_buf, 7);

    TX_buf[2] = '4';//Device ID 4
    UART_LC_4->write(TX_buf, 7);
}

void LCIND_class::TX_request() {
    uint8_t TX_buf[8] = {0};
    TX_buf[0] = 0x02;
    TX_buf[1] = '0';
    TX_buf[2] = '1';//Device ID 1
    TX_buf[3] = 'R';
    TX_buf[4] = 'C';
    TX_buf[5] = 'W';
    TX_buf[6] = 'T';
    TX_buf[7] = 0x03;
    UART_LC_1->write(TX_buf, 7);

    TX_buf[2] = '2';//Device ID 2
    UART_LC_2->write(TX_buf, 7);

    TX_buf[2] = '3';//Device ID 3
    UART_LC_3->write(TX_buf, 7);

    TX_buf[2] = '4';//Device ID 4
    UART_LC_4->write(TX_buf, 7);
}

bool LCIND_class::isdigit(uint8_t ASCIIinput) const {
    if( (ASCIIinput >= '0') && (ASCIIinput <= '9') ){
        return true;
    } else {
        return false;
    }
}
/* Test Example
int main() {
    RingBuffer rb;

    // 예시 데이터 1개 프레임
    uint8_t packet[] = {
        0x02, '0', '1', 'R', 'C', 'W', 'T',
        'S', 'N', 'P', '2', '-', '0','0','1','2','3','4',
        'k','g', 0x03
    };

    // 바이트 단위로 연속 수신 처리 시뮬레이션
    process_stream(rb, packet, sizeof(packet));

    return 0;
}
*/
namespace AP 
{
LCIND_class *LCIND_g()
{
    return LCIND_class::get_singleton();

}
};