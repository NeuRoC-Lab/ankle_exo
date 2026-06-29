#pragma once

class SerialMotorControl  {
public:
    SerialMotorControl(Stream& serial, MotorCmd& cmd, CANController& motor)
        : m_serial(serial),
          m_cmd(cmd),
          m_controller(motor)
    {}

    void update();

private:
    Stream& m_serial;
    MotorCmd& m_cmd;
    String m_line;
    CANController& m_controller;

    void handleLine(String line);
    void handleSetCommand(String line);

    bool tokenize(String line, String tok[], int& n, int max_tokens);
    bool getParam(const String tok[], int n, const String& name, float& value);
    bool getTargetId(const String tok[], int n, uint32_t& target_id);

#if defined(MIT_MODE)
    void applyMITParam(const String& name, float value);
    void printMITCommand();
#elif defined(SERVO_MODE)
    void applyServoCommand(const String tok[], int n);
    void printServoCommand();
#endif
};

void SerialMotorControl::update() {
    while (m_serial.available()) {
        char c = m_serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            handleLine(m_line);
            m_line = "";
        } else {
            m_line += c;
        }
    }
}

void SerialMotorControl::handleLine(String line) {
    line.trim();
    line.toLowerCase();

    if (line.length() == 0) {
        return;
    }

#if defined(MIT_MODE)
    if (line.startsWith("stop")) {
        motor.m_enabled = false;
        delay(100);
        m_controller.sendMessage(exitMotorMode);
        m_serial.println("Stopping MIT mode");
        return;
    }

    if (line.startsWith("start")) {
        m_serial.println("Starting MIT mode");
        motor.m_enabled = true;
        delay(100);
        m_controller.sendMessage(enterMotorMode);
        return;
    }

    if (line.startsWith("zero")) {
        delay(100);
        m_serial.println("Setting MIT zero position");
        m_controller.sendMessage(setZeroPosition);
        return;
    }
#elif defined(SERVO_MODE)
    if (line.startsWith("stop")) {
        m_serial.println("Servo stop requested: sending duty 0");
        m_controller.can_set_duty(0.0f);
        return;
    }
#endif

    if (line.startsWith("set ")) {
        handleSetCommand(line);
        return;
    }

#if defined(MIT_MODE)
    m_serial.println("usage: start | stop | zero | set id 1 pos 1.0 vel 0.0 kp 10.0 kd 0.5 trq 0.0");
#elif defined(SERVO_MODE)
    m_serial.println("usage: set id 1 duty 0.1 | current 2.0 | brake 3.0 | rpm 10000 | pos 180 | pos 180 rpm 30000 acc 60000 | origin 0");
#endif
}

bool SerialMotorControl::tokenize(String line, String tok[], int& n, int max_tokens) {
    n = 0;
    line.trim();

    while (line.length() && n < max_tokens) {
        int sp = line.indexOf(' ');

        if (sp < 0) {
            tok[n++] = line;
            break;
        }

        String part = line.substring(0, sp);
        part.trim();

        if (part.length() > 0) {
            tok[n++] = part;
        }

        line = line.substring(sp + 1);
        line.trim();
    }

    return n > 0;
}

bool SerialMotorControl::getParam(const String tok[], int n, const String& name, float& value) {
    // Expected format:
    // set id 1 pos 1.0 vel 0.0 ...
    //
    // tok[0] = "set"
    // tok[1] = "id"
    // tok[2] = "1"
    // tok[3] = "pos"
    // tok[4] = "1.0"

    for (int i = 1; i + 1 < n; i += 2) {
        if (tok[i] == name) {
            value = tok[i + 1].toFloat();
            return true;
        }
    }

    return false;
}

bool SerialMotorControl::getTargetId(const String tok[], int n, uint32_t& target_id) {
    float id_value = 0.0f;

    if (!getParam(tok, n, "id", id_value)) {
        return false;
    }

    target_id = (uint32_t)id_value;
    return true;
}

void SerialMotorControl::handleSetCommand(String line) {
    String tok[16];
    int n = 0;

    if (!tokenize(line, tok, n, 16)) {
        return;
    }

    if (n < 3 || tok[0] != "set") {
        return;
    }

    // After "set", we need key/value pairs.
    // So n - 1 must be even.
    if (((n - 1) % 2) != 0) {
#if defined(MIT_MODE)
        m_serial.println("usage: set id 1 pos 1.0 vel 0.0 kp 10.0 kd 0.5 trq 0.0");
#elif defined(SERVO_MODE)
        m_serial.println("usage: set id 1 duty 0.1 | current 2.0 | brake 3.0 | rpm 10000 | pos 180 | pos 180 rpm 30000 acc 60000");
#endif
        return;
    }

    uint32_t target_id = 0;

    if (!getTargetId(tok, n, target_id)) {
        m_serial.println("error: missing id");
        return;
    }

    if (target_id != m_controller.m_canId) {
        // Command is for another motor. Silently ignore it.
        return;
    }

#if defined(MIT_MODE)
    for (int i = 1; i + 1 < n; i += 2) {
        String name = tok[i];

        if (name == "id") {
            continue;
        }

        float value = tok[i + 1].toFloat();
        applyMITParam(name, value);
    }

    printMITCommand();

#elif defined(SERVO_MODE)
    applyServoCommand(tok, n);
#endif
}

#if defined(MIT_MODE)

void SerialMotorControl::applyMITParam(const String& name, float value) {
    if (name == "pos") {
        m_cmd.position = value;
    } else if (name == "vel") {
        m_cmd.velocity = value;
    } else if (name == "trq") {
        m_cmd.torque = value;
    } else if (name == "kp") {
        m_cmd.kp = value;
    } else if (name == "kd") {
        m_cmd.kd = value;
    } else {
        m_serial.print("bad MIT param: ");
        m_serial.println(name);
    }
}

void SerialMotorControl::printMITCommand() {
    m_serial.print("MIT cmd id=");
    m_serial.print(m_controller.m_canId);

    m_serial.print(" pos=");
    m_serial.print(m_cmd.position, 4);

    m_serial.print(" vel=");
    m_serial.print(m_cmd.velocity, 4);

    m_serial.print(" kp=");
    m_serial.print(m_cmd.kp, 4);

    m_serial.print(" kd=");
    m_serial.print(m_cmd.kd, 4);

    m_serial.print(" trq=");
    m_serial.println(m_cmd.torque, 4);
}

#elif defined(SERVO_MODE)

void SerialMotorControl::applyServoCommand(const String tok[], int n) {
    float duty = 0.0f;
    float current = 0.0f;
    float brake = 0.0f;
    float rpm = 0.0f;
    float pos = 0.0f;
    float acc = 0.0f;
    float origin = 0.0f;

    bool has_duty = getParam(tok, n, "duty", duty);
    bool has_current = getParam(tok, n, "current", current);
    bool has_brake = getParam(tok, n, "brake", brake);
    bool has_rpm = getParam(tok, n, "rpm", rpm);
    bool has_pos = getParam(tok, n, "pos", pos);
    bool has_acc = getParam(tok, n, "acc", acc);
    bool has_origin = getParam(tok, n, "origin", origin);

    // Optional aliases.
    if (!has_current) {
        has_current = getParam(tok, n, "cur", current);
    }

    if (!has_brake) {
        has_brake = getParam(tok, n, "brk", brake);
    }

    if (!has_rpm) {
        has_rpm = getParam(tok, n, "vel", rpm);
    }

    if (!has_acc) {
        has_acc = getParam(tok, n, "accel", acc);
    }

    int command_count = 0;

    if (has_duty) {
        command_count++;
    }

    if (has_current) {
        command_count++;
    }

    if (has_brake) {
        command_count++;
    }

    if (has_origin) {
        command_count++;
    }

    // Position-speed counts as one command.
    // Plain position counts as one command.
    // Plain rpm counts as one command.
    if (has_pos && has_rpm && has_acc) {
        command_count++;
    } else {
        if (has_pos) {
            command_count++;
        }

        if (has_rpm) {
            command_count++;
        }
    }

    if (command_count != 1) {
        m_serial.println("error: send exactly one servo command");
        m_serial.println("examples:");
        m_serial.println("  set id 1 duty 0.1");
        m_serial.println("  set id 1 current 2.0");
        m_serial.println("  set id 1 brake 3.0");
        m_serial.println("  set id 1 rpm 10000");
        m_serial.println("  set id 1 pos 180");
        m_serial.println("  set id 1 pos 180 rpm 30000 acc 60000");
        m_serial.println("  set id 1 origin 0");
        return;
    }

    if (has_duty) {
        m_controller.can_set_duty(duty);
    } else if (has_current) {
        m_controller.can_set_current(current);
    } else if (has_brake) {
        m_controller.can_set_current_brake(brake);
    } else if (has_origin) {
        m_controller.can_set_origin((uint8_t)origin);
    } else if (has_pos && has_rpm && has_acc) {
        m_controller.can_set_position_speed(pos, rpm, acc);
    } else if (has_pos) {
        m_controller.can_set_position(pos);
    } else if (has_rpm) {
        m_controller.can_set_rpm(rpm);
    }

    printServoCommand();
}

void SerialMotorControl::printServoCommand() {
    m_serial.print("Servo cmd id=");
    m_serial.print(m_controller.m_canId);

    m_serial.print(" packet=");
    m_serial.print((uint32_t)m_cmd.packetID);

    m_serial.print(" len=");
    m_serial.print(m_cmd.len);

    m_serial.print(" data=");

    for (uint8_t i = 0; i < m_cmd.len; i++) {
        if (m_cmd.data[i] < 0x10) {
            m_serial.print("0");
        }

        m_serial.print(m_cmd.data[i], HEX);

        if (i + 1 < m_cmd.len) {
            m_serial.print(" ");
        }
    }

    m_serial.println();
}

#endif  // defined(SERVO_MODE)