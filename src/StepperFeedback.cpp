#include <Arduino.h>

// flags
#define I2C_FLAGS_READ    0b00000001
#define I2C_FLAGS_DONE    0b00000010
#define I2C_FLAGS_ERROR   0b00000100
#define I2C_FLAGS_RESTART 0b00001000

#define CPR 4095 // counts per revolution
#define MMPR 100 // millimeters per revolution
#define MAX_LENGTH 256 // chars for serial communication
#define Kp 2

char send_buf[MAX_LENGTH];
char recv_buf[MAX_LENGTH];
char setpoint_buf[MAX_LENGTH];

int16_t posRaw = 0;
int16_t motorPosOffset = 0;

float multiplier = 0;

long motorPos = 0;
long motorPosPrev = 0;
long setpointPosition = 4096;
long setpointVelocity = 0;
long setpointAcceleration = 0;

bool running = false;
bool initalizePosition = true;
bool newCommand = false;

long errPos = 0;
bool errIsNegative = false;

volatile uint16_t send_len = 0;
volatile uint16_t recv_len = 0;
volatile uint8_t err_status = 0;
volatile bool sending = false;
volatile bool update = false;
volatile bool controlUpdate = false;
volatile bool newSetpoint = false;

typedef struct i2c_transaction_t {
    volatile uint8_t addr;
    volatile uint8_t n_bytes;
    volatile uint8_t buf[32];
    volatile uint8_t flags;
    volatile uint8_t err_code;
    i2c_transaction_t *next;
};

i2c_transaction_t *cur_transaction;

void startI2C(i2c_transaction_t *transaction) {
    cur_transaction = transaction;
    TWBR = 12; // 400 kHz
    TWCR = 0b10100101; // initializes and starts an I2C transaction
}

void phaseUnwrap(const long current, long* previous, long* position) {
    int delta = current - *previous;

    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }

    *position += delta;
    *previous = current;
}

void ccw() {
    PORTB |= (1 << DDB0);
}

void cw() {
    PORTB &= ~(1 << DDB0);
}

void setStepFrequency(long stepHz) {
    if (stepHz <= 0) {
        TCCR1A &= ~(1 << COM1A0);
        PORTB &= ~(1 << PORTB1);
        return;
    }

    TCCR1A |= (1 << COM1A0);

    // CTC toggle mode:
    // stepHz = F_CPU / (2 * prescaler * (OCR1A + 1))
    uint32_t newOCR = (F_CPU / (2UL * 64UL * (uint32_t)stepHz)) - 1UL;

    if (newOCR > 65535) {
        newOCR = 65535;
    }

    if (newOCR < 1) {
        newOCR = 1;
    }

    cli();
    if (TCNT1 > newOCR) {
        TCNT1 = 0;
    }

    OCR1A = (uint16_t)newOCR;
    sei();
}

void manualPrint(const char *str) {
    if (!sending) {
        send_len = strlen(str);
        strncpy(send_buf, str, MAX_LENGTH - 1);
        send_buf[MAX_LENGTH - 1] = '\0';

        cli();
        err_status = 0;
        sending = true;
        sei();

        UCSR0B |= (1 << UDRIE0);
    }
}

void manualPrintln(const char *str) {
    if (!sending) {
        size_t str_len = strlen(str);

        if (str_len > MAX_LENGTH - 3) {
            str_len = MAX_LENGTH - 3;
        }

        strncpy(send_buf, str, str_len);
        send_buf[str_len++] = '\r';
        send_buf[str_len++] = '\n';
        send_buf[str_len] = '\0';

        send_len = str_len;

        cli();
        err_status = 0;
        sending = true;
        sei();

        UCSR0B |= (1 << UDRIE0);
    }
}

void readEncoder() {
    // setup for encoder position
    i2c_transaction_t read_Data[2];

    // write the address to read from
    read_Data[0].addr = 0x36; // device address
    read_Data[0].n_bytes = 1;
    read_Data[0].buf[0] = 0x0C; // raw position high byte
    read_Data[0].flags = I2C_FLAGS_RESTART;
    read_Data[0].next = &(read_Data[1]);

    // read from the address
    read_Data[1].addr = 0x36;
    read_Data[1].n_bytes = 2; // number of bytes to sequentially read
    read_Data[1].flags = I2C_FLAGS_READ;

    startI2C(read_Data);

    while (!(read_Data[1].flags & I2C_FLAGS_DONE));

    if (read_Data[0].flags & I2C_FLAGS_ERROR) {
    }
    else if (read_Data[1].flags & I2C_FLAGS_ERROR) {
    }
    else {
        posRaw = ((int16_t)(read_Data[1].buf[0] << 8) | read_Data[1].buf[1]);

        if (initalizePosition) {
            initalizePosition = false;
            motorPosPrev = posRaw;
            motorPos = 0;
            motorPosOffset = posRaw;
        } else {
            phaseUnwrap(posRaw, &motorPosPrev, &motorPos);
        }
    }
}

void printDiagnostics() {
    char fmtdStr[64];
    char cmdVelStr[32];
    char multiplierStr[32];

    // CTC toggle mode:
    // output frequency = F_CPU / (2 * prescaler * (OCR1A + 1))
    dtostrf(F_CPU / (2.0 * 64.0 * (OCR1A + 1.0)), 0, 2, cmdVelStr);
    dtostrf(multiplier, 0, 2, multiplierStr);

    if (errIsNegative) {
        snprintf(
            fmtdStr,
            64,
            "Dir: CCW | Pos Err: %ld | Cmd Vel: %s | Multiplier: %s",
            errPos,
            cmdVelStr,
            multiplierStr
        );
    } else {
        snprintf(
            fmtdStr,
            64,
            "Dir: CW | Pos Err: %ld | Cmd Vel: %s | Multiplier: %s",
            errPos,
            cmdVelStr,
            multiplierStr
        );
    }

    manualPrintln(fmtdStr);
}

void setup() {
    cli();

    // Timer1 CTC Setup, toggles D9 / OC1A
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    TCCR1A |= (1 << COM1A0);
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10);
    OCR1A = 0;

    // Timer2 Setup, run ~100 Hz ISR
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    TCCR2A |= (1 << WGM21);
    OCR2A = 155;
    TIMSK2 |= (1 << OCIE2A);
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

    // Set pins D8 and D9 as outputs
    DDRB = (1 << DDB0) | (1 << DDB1);

    // UART setup
    // 500000 baud
    UBRR0H = 0;
    UBRR0L = 1;
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0)  |
             (1 << TXEN0)  |
             (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    sei();
}

void loop() {
    readEncoder();

    // read string from serial
    if (update) {
        update = false;
        running = true;
        cli();
        strcpy(setpoint_buf, recv_buf);
        sei();

        multiplier = 0;

        manualPrintln(setpoint_buf);

        setpointPosition = strtol(setpoint_buf, NULL, 10);
    }

    if (controlUpdate && running) {
        controlUpdate = false;
        if (errPos == 0) {
            running = false;
            manualPrintln("GOAL REACHED");
        }
        multiplier += 0.004;

        if (multiplier > 1) {
            multiplier = 1;
        }

        // feedback control
        errPos = setpointPosition - motorPos;
        errIsNegative = errPos < 0;

        long cmdVel = abs(multiplier * errPos);
        long cmdVelLimited = (long)(Kp * constrain(cmdVel, 0, 2000));

        cmdVelLimited = constrain(cmdVelLimited, 1, 2000);

        if (errIsNegative) {
            ccw();
        } else {
            cw();
        }

        setStepFrequency(cmdVelLimited);
        printDiagnostics();
    }
}

ISR(TWI_vect) {
    static uint8_t pos = 0;

    switch (TWSR & 0xF8) {
        case 0x08: // start bit sent
        case 0x10:
            pos = 0;
            TWDR = cur_transaction->addr << 1 |
                   (cur_transaction->flags & I2C_FLAGS_READ ? 1 : 0);
            TWCR = 0b10000101;
            break;

        case 0x18: // ADDR+W sent, ACK received
        case 0x28: // DATA sent, ACK received
            if (pos < cur_transaction->n_bytes) {
                TWDR = cur_transaction->buf[pos++];
                TWCR = 0b10000101;
            } else if (cur_transaction->flags & I2C_FLAGS_RESTART) {
                cur_transaction->flags |= I2C_FLAGS_DONE;
                cur_transaction = cur_transaction->next;
                TWCR = 0b10100101;
            } else {
                cur_transaction->flags |= I2C_FLAGS_DONE;
                TWCR = 0b10010101;
            }
            break;

        case 0x50: // DATA received, ACK sent
            cur_transaction->buf[pos++] = TWDR;
            // fall through intentionally

        case 0x40: // ADDR+R sent, ACK received
            if (pos == cur_transaction->n_bytes - 1) {
                TWCR = 0b10000101;
            } else {
                TWCR = 0b11000101;
            }
            break;

        case 0x58: // DATA received, NACK sent
            cur_transaction->buf[pos] = TWDR;
            cur_transaction->flags |= I2C_FLAGS_DONE;

            if (cur_transaction->flags & I2C_FLAGS_RESTART) {
                cur_transaction = cur_transaction->next;
                TWCR = 0b10100101;
            } else {
                TWCR = 0b10010101;
            }
            break;

        case 0x20: // ADDR+W sent, NACK received
        case 0x30: // DATA sent, NACK received
        case 0x48: // ADDR+R sent, NACK received
        default:
            pos = 0;
            cur_transaction->err_code = TWSR & 0xF8;
            cur_transaction->flags |= I2C_FLAGS_DONE | I2C_FLAGS_ERROR;
            TWCR = 0b10010000;
            break;
    }
}

ISR(TIMER2_COMPA_vect) {
    controlUpdate = true;
}

ISR(USART_RX_vect) {
    char c = UDR0;

    if (c == '\n') {
        recv_buf[recv_len] = '\0';
        update = true;
        recv_len = 0;
        return;
    }

    if (recv_len < MAX_LENGTH - 1) {
        recv_buf[recv_len++] = c;
        recv_buf[recv_len] = '\0';
    } else {
        err_status = 1;
    }
}

ISR(USART_UDRE_vect) {
    static uint16_t idx = 0;

    if (idx < send_len) {
        UDR0 = send_buf[idx];
        idx++;
    } else {
        sending = false;
        idx = 0;
        UCSR0B &= ~(1 << UDRIE0);
    }
}