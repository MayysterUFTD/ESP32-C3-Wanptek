#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "WanptekProtocol.h"

#include <cmath>
#include <cstdlib>

namespace
{

  constexpr uint8_t PSU_ADDRESS = 0x00;
  constexpr uint32_t PSU_BAUD_RATE = 19200;
  constexpr int PSU_RX_PIN = 3;
  constexpr int PSU_TX_PIN = 4;
  constexpr uint16_t DEFAULT_POLL_INTERVAL_MS = 100;
  constexpr uint16_t MIN_POLL_INTERVAL_MS = 10;
  constexpr uint16_t MAX_POLL_INTERVAL_MS = 1000;
  constexpr uint32_t MODBUS_TIMEOUT_MS = 300;
  constexpr uint32_t SETPOINT_VERIFY_TIMEOUT_MS = 3000;
  constexpr uint32_t MODBUS_SILENCE_US = (35000000UL + PSU_BAUD_RATE - 1) / PSU_BAUD_RATE;
  constexpr uint32_t AP_STATUS_INTERVAL_MS = 10000;
  constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
  constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 30000;
  constexpr uint32_t WIFI_COMMAND_DELAY_MS = 1000;
  constexpr uint32_t AP_HANDOVER_DELAY_MS = 15000;
  constexpr uint16_t PSU_REGISTER_COUNT = 8;

#ifndef WANPTEK_AP_PASSWORD
#error "Set WANPTEK_AP_PASSWORD in config.local.ini"
#endif

  HardwareSerial modbusSerial(1);
  AsyncWebServer server(80);
  Preferences wifiPreferences;
  Preferences settingsPreferences;
  portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

  struct PsuState
  {
    bool connected = false;
    bool outputOn = false;
    bool ocpOn = false;
    bool remoteControl = false;
    bool constantCurrent = false;
    bool alarm = false;
    float voltage = 0.0f;
    float current = 0.0f;
    float setVoltage = 0.0f;
    float setCurrent = 0.0f;
    float maxVoltage = 0.0f;
    float maxCurrent = 0.0f;
    uint32_t sampleSequence = 0;
    uint32_t sampleAt = 0;
    uint32_t sampleInterval = 0;
    uint8_t rawStatus = 0;
    uint8_t voltageFormat = 0;
    uint8_t currentFormat = 0;
    uint16_t rawSetVoltage = 0;
    uint16_t rawSetCurrent = 0;
  };

  enum class CommandType : uint8_t
  {
    None,
    SetValues,
    SetOutput,
    SetOcp,
    SetRemote
  };

  struct PendingCommand
  {
    CommandType type = CommandType::None;
    float voltage = 0.0f;
    float current = 0.0f;
    bool enabled = false;
    uint32_t id = 0;
  };

  enum class CommandProgress : uint8_t
  {
    Idle,
    Queued,
    Verifying,
    Acknowledged,
    Confirmed,
    Failed
  };

  struct CommandStatus
  {
    uint32_t id = 0;
    CommandProgress progress = CommandProgress::Idle;
    const char *error = "";
  };

  struct WifiCredentials
  {
    char ssid[33];
    char password[64];
  };

  enum class WifiCommandType : uint8_t
  {
    None,
    Connect,
    Forget
  };

  struct WifiCommand
  {
    WifiCommandType type = WifiCommandType::None;
    WifiCredentials credentials = {};
    uint32_t queuedAt = 0;
  };

  struct WifiStatus
  {
    bool connected = false;
    bool connecting = false;
    bool apStarted = false;
    char ssid[33] = {};
    char ip[16] = {};
    const char *error = "";
  };

  PsuState state;
  PendingCommand pendingCommand;
  bool commandPending = false;
  bool commandBusy = false;
  CommandStatus commandStatus;
  PendingCommand verifyingCommand;
  bool verifyingSetpoints = false;
  uint32_t verifyStarted = 0;
  uint32_t lastPoll = 0;
  uint32_t lastModbusActivity = 0;
  uint16_t pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
  uint32_t lastApStatus = 0;
  bool accessPointStarted = false;
  String accessPointSsid;
  const char *accessPointError = "Access point was not started";
  const char *lastError = "Waiting for a response from the power supply";
  WifiCredentials savedWifi = {};
  WifiCommand pendingWifi;
  WifiStatus wifiStatus;
  bool wifiCommandPending = false;
  bool wifiConnecting = false;
  bool wifiConnected = false;
  bool wifiStorageReady = false;
  bool settingsStorageReady = false;
  uint32_t wifiAttemptStarted = 0;
  uint32_t lastWifiAttempt = 0;
  uint32_t wifiConnectedAt = 0;
  const char *wifiError = "";

  void setLastError(const char *error)
  {
    portENTER_CRITICAL(&stateMux);
    lastError = error;
    portEXIT_CRITICAL(&stateMux);
  }

  const char *copyLastError()
  {
    const char *error;
    portENTER_CRITICAL(&stateMux);
    error = lastError;
    portEXIT_CRITICAL(&stateMux);
    return error;
  }

  void updateCommandStatus(CommandProgress progress, const char *error = "")
  {
    portENTER_CRITICAL(&stateMux);
    commandStatus.progress = progress;
    commandStatus.error = error;
    if (progress == CommandProgress::Acknowledged ||
        progress == CommandProgress::Confirmed || progress == CommandProgress::Failed)
    {
      commandBusy = false;
    }
    portEXIT_CRITICAL(&stateMux);
  }

  CommandStatus copyCommandStatus()
  {
    CommandStatus snapshot;
    portENTER_CRITICAL(&stateMux);
    snapshot = commandStatus;
    portEXIT_CRITICAL(&stateMux);
    return snapshot;
  }

  uint16_t modbusCrc(const uint8_t *data, size_t length)
  {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
      crc ^= data[i];
      for (uint8_t bit = 0; bit < 8; ++bit)
      {
        crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
      }
    }
    return crc;
  }

  void appendCrc(uint8_t *frame, size_t lengthWithoutCrc)
  {
    const uint16_t crc = modbusCrc(frame, lengthWithoutCrc);
    frame[lengthWithoutCrc] = static_cast<uint8_t>(crc & 0xFF);
    frame[lengthWithoutCrc + 1] = static_cast<uint8_t>(crc >> 8);
  }

  bool validCrc(const uint8_t *frame, size_t length)
  {
    if (length < 3)
    {
      return false;
    }
    const uint16_t received =
        static_cast<uint16_t>(frame[length - 2]) |
        (static_cast<uint16_t>(frame[length - 1]) << 8);
    return modbusCrc(frame, length - 2) == received;
  }

  bool readResponse(uint8_t *response, size_t expectedLength)
  {
    const uint32_t started = millis();
    size_t received = 0;
    while (received < expectedLength &&
           static_cast<uint32_t>(millis() - started) < MODBUS_TIMEOUT_MS)
    {
      while (modbusSerial.available() && received < expectedLength)
      {
        response[received++] = static_cast<uint8_t>(modbusSerial.read());
      }
      if (received < expectedLength)
      {
        delay(1);
      }
    }
    lastModbusActivity = micros();
    return received == expectedLength;
  }

  void waitForModbusSilence()
  {
    const uint32_t elapsed = micros() - lastModbusActivity;
    if (elapsed < MODBUS_SILENCE_US)
    {
      delayMicroseconds(MODBUS_SILENCE_US - elapsed);
    }
  }

  void discardPendingBytes()
  {
    while (modbusSerial.available())
    {
      modbusSerial.read();
    }
  }

  bool readRegisters(uint16_t start, uint16_t count, uint16_t *registers)
  {
    uint8_t request[8] = {
        PSU_ADDRESS, 0x03, static_cast<uint8_t>(start >> 8),
        static_cast<uint8_t>(start), static_cast<uint8_t>(count >> 8),
        static_cast<uint8_t>(count), 0, 0};
    appendCrc(request, 6);

    waitForModbusSilence();
    discardPendingBytes();
    modbusSerial.write(request, sizeof(request));
    modbusSerial.flush();

    uint8_t response[5 + PSU_REGISTER_COUNT * 2];
    if (!readResponse(response, sizeof(response)))
    {
      setLastError("No Modbus response");
      return false;
    }
    if (response[0] != PSU_ADDRESS || response[1] != 0x03 ||
        response[2] != count * 2 || !validCrc(response, sizeof(response)))
    {
      setLastError("Invalid Modbus response");
      return false;
    }

    for (uint16_t i = 0; i < count; ++i)
    {
      registers[i] = (static_cast<uint16_t>(response[3 + i * 2]) << 8) |
                     response[4 + i * 2];
    }
    return true;
  }

  uint16_t swapBytes(uint16_t value)
  {
    return static_cast<uint16_t>((value << 8) | (value >> 8));
  }

  bool writePsuState(const PsuState &newState)
  {
    if (!std::isfinite(newState.setVoltage) || !std::isfinite(newState.setCurrent) ||
        newState.setVoltage < 0.0f || newState.setCurrent < 0.0f ||
        newState.setVoltage > 655.35f || newState.setCurrent > 65.535f)
    {
      setLastError("Setpoints exceed the Modbus write range");
      return false;
    }
    const uint16_t voltageRaw =
        static_cast<uint16_t>(lroundf(newState.setVoltage * 100.0f));
    const uint16_t currentRaw =
        static_cast<uint16_t>(lroundf(newState.setCurrent * 1000.0f));

    auto request = makePsuWriteRequest(PSU_ADDRESS, newState.outputOn,
                                       newState.ocpOn, newState.remoteControl,
                                       voltageRaw, currentRaw,
                                       (newState.rawStatus & 0x08) != 0);
    appendCrc(request.data(), request.size() - 2);

    Serial.print("Modbus FC16 TX:");
    for (uint8_t byte : request)
    {
      Serial.printf(" %02X", static_cast<unsigned>(byte));
    }
    Serial.println();

    waitForModbusSilence();
    discardPendingBytes();
    modbusSerial.write(request.data(), request.size());
    modbusSerial.flush();

    uint8_t response[8];
    if (!readResponse(response, sizeof(response)))
    {
      setLastError("No write acknowledgement");
      return false;
    }
    Serial.print("Modbus FC16 RX:");
    for (uint8_t byte : response)
    {
      Serial.printf(" %02X", static_cast<unsigned>(byte));
    }
    Serial.println();
    if (response[0] != PSU_ADDRESS || response[1] != 0x10 ||
        response[2] != 0x00 || response[3] != 0x00 ||
        response[4] != 0x00 || response[5] != 0x03 ||
        !validCrc(response, sizeof(response)))
    {
      setLastError("Invalid write acknowledgement");
      return false;
    }
    return true;
  }

  PsuState copyState()
  {
    PsuState snapshot;
    portENTER_CRITICAL(&stateMux);
    snapshot = state;
    portEXIT_CRITICAL(&stateMux);
    return snapshot;
  }

  void storeState(const PsuState &newState)
  {
    portENTER_CRITICAL(&stateMux);
    state = newState;
    portEXIT_CRITICAL(&stateMux);
  }

  void logSetpointReadback(const PsuState &observed)
  {
    Serial.printf("Setpoints: requested %.2f V / %.3f A, read %.2f V / %.3f A, "
                  "status=%02X formatV=%02X formatI=%02X registers=%04X %04X\n",
                  verifyingCommand.voltage, verifyingCommand.current,
                  observed.setVoltage, observed.setCurrent,
                  static_cast<unsigned>(observed.rawStatus),
                  static_cast<unsigned>(observed.voltageFormat),
                  static_cast<unsigned>(observed.currentFormat),
                  static_cast<unsigned>(observed.rawSetVoltage),
                  static_cast<unsigned>(observed.rawSetCurrent));
  }

  void finishSetpointVerification(const PsuState &observed, bool confirmed)
  {
    logSetpointReadback(observed);
    verifyingSetpoints = false;
    if (confirmed)
    {
      updateCommandStatus(CommandProgress::Confirmed);
    }
    else
    {
      updateCommandStatus(CommandProgress::Failed,
                          "Power supply did not confirm setpoints in readback");
    }
  }

  void pollPsu()
  {
    uint16_t registers[PSU_REGISTER_COUNT] = {};
    if (!readRegisters(0, PSU_REGISTER_COUNT, registers))
    {
      PsuState currentState = copyState();
      currentState.connected = false;
      storeState(currentState);
      return;
    }

    const uint8_t status = static_cast<uint8_t>(registers[0] >> 8);
    const uint8_t voltageFormat = static_cast<uint8_t>(registers[0]);
    const uint8_t currentFormat = static_cast<uint8_t>(registers[1] >> 8);
    const bool bigEndian = (status & 0x08) != 0;
    const float voltageScale = (voltageFormat & 0x80) ? 0.1f : 0.01f;
    const float currentScale = (currentFormat & 0x80) ? 0.01f : 0.001f;

    auto decodedRegister = [bigEndian](uint16_t value)
    {
      return bigEndian ? value : swapBytes(value);
    };

    // The reference protocol treats remote control as a local write setting,
    // not as a read-back status bit.
    PsuState newState = copyState();
    newState.connected = true;
    newState.outputOn = (status & 0x01) != 0;
    newState.ocpOn = (status & 0x02) != 0;
    newState.constantCurrent = (status & 0x10) != 0;
    newState.alarm = (status & 0x20) != 0;
    newState.voltage = decodedRegister(registers[2]) * voltageScale;
    newState.current = decodedRegister(registers[3]) * currentScale;
    newState.setVoltage = decodedRegister(registers[4]) * voltageScale;
    newState.setCurrent = decodedRegister(registers[5]) * currentScale;
    newState.maxVoltage = decodedRegister(registers[6]) * voltageScale;
    newState.maxCurrent = decodedRegister(registers[7]) * currentScale;
    newState.rawStatus = status;
    newState.voltageFormat = voltageFormat;
    newState.currentFormat = currentFormat;
    newState.rawSetVoltage = registers[4];
    newState.rawSetCurrent = registers[5];
    const uint32_t now = millis();
    newState.sampleInterval = newState.sampleSequence ? now - newState.sampleAt : 0;
    newState.sampleAt = now;
    ++newState.sampleSequence;
    storeState(newState);
    setLastError("");
    if (verifyingSetpoints &&
        fabsf(newState.setVoltage - verifyingCommand.voltage) < 0.0051f &&
        fabsf(newState.setCurrent - verifyingCommand.current) < 0.00051f)
    {
      finishSetpointVerification(newState, true);
    }
  }

  bool parseNumber(AsyncWebServerRequest *request, const char *name,
                   float &value)
  {
    if (!request->hasParam(name))
    {
      return false;
    }
    const String raw = request->getParam(name)->value();
    char *end = nullptr;
    const float parsed = strtof(raw.c_str(), &end);
    if (end == raw.c_str() || *end != '\0' || !std::isfinite(parsed))
    {
      return false;
    }
    value = parsed;
    return true;
  }

  bool parseEnabled(AsyncWebServerRequest *request, bool &enabled)
  {
    if (!request->hasParam("enabled"))
    {
      return false;
    }
    const String raw = request->getParam("enabled")->value();
    if (raw == "1")
    {
      enabled = true;
      return true;
    }
    if (raw == "0")
    {
      enabled = false;
      return true;
    }
    return false;
  }

  bool queueCommand(AsyncWebServerRequest *request,
                    const PendingCommand &command)
  {
    uint32_t id;
    portENTER_CRITICAL(&stateMux);
    if (commandBusy)
    {
      portEXIT_CRITICAL(&stateMux);
      request->send(409, "application/json",
                    "{\"error\":\"Previous command is still in progress\"}");
      return false;
    }
    ++commandStatus.id;
    if (commandStatus.id == 0)
    {
      ++commandStatus.id;
    }
    id = commandStatus.id;
    commandStatus.progress = CommandProgress::Queued;
    commandStatus.error = "";
    pendingCommand = command;
    pendingCommand.id = id;
    commandPending = true;
    commandBusy = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(202, "application/json",
                  String("{\"queued\":true,\"id\":") + String(id) + "}");
    return true;
  }

  const char *commandProgressName(CommandProgress progress)
  {
    switch (progress)
    {
    case CommandProgress::Idle: return "idle";
    case CommandProgress::Queued: return "queued";
    case CommandProgress::Verifying: return "verifying";
    case CommandProgress::Acknowledged: return "acknowledged";
    case CommandProgress::Confirmed: return "confirmed";
    case CommandProgress::Failed: return "failed";
    }
    return "failed";
  }

  String stateJson()
  {
    const PsuState snapshot = copyState();
    String json;
    json.reserve(320);
    json = "{\"connected\":";
    json += snapshot.connected ? "true" : "false";
    json += ",\"outputOn\":";
    json += snapshot.outputOn ? "true" : "false";
    json += ",\"ocpOn\":";
    json += snapshot.ocpOn ? "true" : "false";
    json += ",\"remoteControl\":";
    json += snapshot.remoteControl ? "true" : "false";
    json += ",\"constantCurrent\":";
    json += snapshot.constantCurrent ? "true" : "false";
    json += ",\"alarm\":";
    json += snapshot.alarm ? "true" : "false";
    json += ",\"voltage\":";
    json += String(snapshot.voltage, 2);
    json += ",\"current\":";
    json += String(snapshot.current, 3);
    json += ",\"setVoltage\":";
    json += String(snapshot.setVoltage, 2);
    json += ",\"setCurrent\":";
    json += String(snapshot.setCurrent, 3);
    json += ",\"maxVoltage\":";
    json += String(snapshot.maxVoltage, 2);
    json += ",\"maxCurrent\":";
    json += String(snapshot.maxCurrent, 3);
    json += ",\"sampleSequence\":";
    json += String(snapshot.sampleSequence);
    json += ",\"sampleIntervalMs\":";
    json += String(snapshot.sampleInterval);
    uint16_t interval;
    portENTER_CRITICAL(&stateMux);
    interval = pollIntervalMs;
    portEXIT_CRITICAL(&stateMux);
    json += ",\"pollIntervalMs\":";
    json += String(interval);
    json += ",\"error\":\"";
    json += copyLastError();
    const CommandStatus status = copyCommandStatus();
    json += "\",\"commandId\":";
    json += String(status.id);
    json += ",\"commandStatus\":\"";
    json += commandProgressName(status.progress);
    json += "\",\"commandError\":\"";
    json += status.error;
    json += "\"}";
    return json;
  }

  void sendState(AsyncWebServerRequest *request)
  {
    request->send(200, "application/json", stateJson());
  }

  void handleCommand()
  {
    PendingCommand command;
    portENTER_CRITICAL(&stateMux);
    if (!commandPending)
    {
      portEXIT_CRITICAL(&stateMux);
      return;
    }
    command = pendingCommand;
    pendingCommand = PendingCommand{};
    commandPending = false;
    portEXIT_CRITICAL(&stateMux);

    PsuState newState = copyState();
    if (!newState.connected)
    {
      setLastError("Power supply is not connected");
      updateCommandStatus(CommandProgress::Failed, "Power supply is not connected");
      return;
    }

    switch (command.type)
    {
    case CommandType::SetValues:
      newState.setVoltage = command.voltage;
      newState.setCurrent = command.current;
      break;
    case CommandType::SetOutput:
      newState.outputOn = command.enabled;
      break;
    case CommandType::SetOcp:
      newState.ocpOn = command.enabled;
      break;
    case CommandType::SetRemote:
      newState.remoteControl = command.enabled;
      break;
    case CommandType::None:
      updateCommandStatus(CommandProgress::Failed, "Invalid command");
      return;
    }

    if (writePsuState(newState))
    {
      setLastError("");
      if (command.type == CommandType::SetValues)
      {
        verifyingCommand = command;
        verifyingSetpoints = true;
        verifyStarted = millis();
        updateCommandStatus(CommandProgress::Verifying);
        lastPoll = millis() - MAX_POLL_INTERVAL_MS;
      }
      else
      {
        storeState(newState);
        updateCommandStatus(CommandProgress::Acknowledged);
      }
    }
    else
    {
      const char *error = copyLastError();
      updateCommandStatus(CommandProgress::Failed, error);
      Serial.println(error);
      PsuState previous = copyState();
      previous.connected = false;
      storeState(previous);
    }
  }

  void printAccessPointStatus()
  {
    if (!accessPointStarted)
    {
      if (wifiConnected)
      {
        Serial.print("Home Wi-Fi: http://");
        Serial.println(WiFi.localIP());
      }
      else if (wifiConnecting)
      {
        Serial.println("Connecting to Wi-Fi");
      }
      else
      {
        Serial.println(accessPointError);
      }
      return;
    }
    Serial.print("Wi-Fi: ");
    Serial.println(accessPointSsid);
    Serial.print("Panel: http://");
    Serial.println(WiFi.softAPIP());
    wifi_config_t config = {};
    const esp_err_t configResult = esp_wifi_get_config(WIFI_IF_AP, &config);
    if (configResult != ESP_OK)
    {
      Serial.printf("Failed to read AP configuration: %s\n", esp_err_to_name(configResult));
      return;
    }
    Serial.printf("AP radio: SSID=%.*s channel=%u hidden=%u\n",
                  static_cast<int>(config.ap.ssid_len),
                  reinterpret_cast<const char *>(config.ap.ssid),
                  static_cast<unsigned>(config.ap.channel),
                  static_cast<unsigned>(config.ap.ssid_hidden));
    uint8_t channel = 0;
    wifi_second_chan_t secondaryChannel;
    const esp_err_t channelResult = esp_wifi_get_channel(&channel, &secondaryChannel);
    if (channelResult != ESP_OK)
    {
      Serial.printf("Failed to read radio channel: %s\n", esp_err_to_name(channelResult));
      return;
    }
    int8_t power = 0;
    const esp_err_t powerResult = esp_wifi_get_max_tx_power(&power);
    if (powerResult != ESP_OK)
    {
      Serial.printf("Failed to read radio power: %s\n", esp_err_to_name(powerResult));
      return;
    }
    Serial.printf("Radio active: channel=%u max power=%.2f dBm\n",
                  static_cast<unsigned>(channel), static_cast<float>(power) / 4.0f);
  }

  bool startAccessPoint()
  {
    if (!WiFi.mode(savedWifi.ssid[0] ? WIFI_AP_STA : WIFI_AP))
    {
      accessPointError = "Failed to initialize Wi-Fi AP mode";
      return false;
    }

    String mac = WiFi.softAPmacAddress();
    mac.replace(":", "");
    accessPointSsid = "Wanptek-" + mac.substring(8);
    if (!WiFi.softAP(accessPointSsid.c_str(), WANPTEK_AP_PASSWORD, 1, 0, 4))
    {
      accessPointError = "Failed to start access point";
      return false;
    }
#ifdef WANPTEK_WIFI_TX_POWER
    if (!WiFi.setTxPower(WANPTEK_WIFI_TX_POWER))
    {
      Serial.println("Failed to set Wi-Fi transmit power");
    }
#endif
    return true;
  }

  void updateWifiStatus()
  {
    WifiStatus snapshot;
    snapshot.connected = wifiConnected;
    snapshot.connecting = wifiConnecting;
    snapshot.apStarted = accessPointStarted;
    strlcpy(snapshot.ssid, savedWifi.ssid, sizeof(snapshot.ssid));
    if (wifiConnected)
    {
      strlcpy(snapshot.ip, WiFi.localIP().toString().c_str(), sizeof(snapshot.ip));
    }
    snapshot.error = wifiError;
    portENTER_CRITICAL(&stateMux);
    wifiStatus = snapshot;
    portEXIT_CRITICAL(&stateMux);
  }

  String jsonString(const char *text)
  {
    String result = "\"";
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(text); *p; ++p)
    {
      if (*p == '"' || *p == '\\')
      {
        result += '\\';
        result += static_cast<char>(*p);
      }
      else if (*p < 0x20)
      {
        char escaped[7];
        snprintf(escaped, sizeof(escaped), "\\u%04x", *p);
        result += escaped;
      }
      else
      {
        result += static_cast<char>(*p);
      }
    }
    result += '"';
    return result;
  }

  String wifiStatusJson()
  {
    WifiStatus snapshot;
    portENTER_CRITICAL(&stateMux);
    snapshot = wifiStatus;
    portEXIT_CRITICAL(&stateMux);
    String json = "{\"connected\":";
    json += snapshot.connected ? "true" : "false";
    json += ",\"connecting\":";
    json += snapshot.connecting ? "true" : "false";
    json += ",\"apStarted\":";
    json += snapshot.apStarted ? "true" : "false";
    json += ",\"ssid\":";
    json += jsonString(snapshot.ssid);
    json += ",\"ip\":";
    json += jsonString(snapshot.ip);
    json += ",\"error\":";
    json += jsonString(snapshot.error);
    json += "}";
    return json;
  }

  void ensureAccessPoint()
  {
    if (!accessPointStarted)
    {
      accessPointStarted = startAccessPoint();
      if (!accessPointStarted)
      {
        wifiError = accessPointError;
        Serial.println(accessPointError);
      }
    }
  }

  void beginWifiConnection()
  {
    if (!WiFi.mode(accessPointStarted ? WIFI_AP_STA : WIFI_STA))
    {
      wifiError = "Failed to initialize Wi-Fi";
      wifiConnecting = false;
      lastWifiAttempt = millis();
      ensureAccessPoint();
      return;
    }
    WiFi.disconnect();
    WiFi.begin(savedWifi.ssid, savedWifi.password[0] ? savedWifi.password : nullptr);
#ifdef WANPTEK_WIFI_TX_POWER
    if (!WiFi.setTxPower(WANPTEK_WIFI_TX_POWER))
    {
      Serial.println("Failed to set Wi-Fi transmit power");
    }
#endif
    wifiConnecting = true;
    wifiConnected = false;
    wifiAttemptStarted = millis();
    lastWifiAttempt = wifiAttemptStarted;
    Serial.println("Connecting to Wi-Fi");
  }

  void handleWifi()
  {
    WifiCommand command;
    bool hasCommand = false;
    const uint32_t now = millis();
    portENTER_CRITICAL(&stateMux);
    if (wifiCommandPending &&
        static_cast<uint32_t>(now - pendingWifi.queuedAt) >= WIFI_COMMAND_DELAY_MS)
    {
      command = pendingWifi;
      pendingWifi = WifiCommand{};
      wifiCommandPending = false;
      hasCommand = true;
    }
    portEXIT_CRITICAL(&stateMux);

    if (hasCommand)
    {
      if (!wifiStorageReady)
      {
        wifiError = "Could not save Wi-Fi settings";
      }
      else if (command.type == WifiCommandType::Connect)
      {
        if (wifiPreferences.putBytes("network", &command.credentials,
                                     sizeof(command.credentials)) != sizeof(command.credentials))
        {
          wifiError = "Could not save Wi-Fi settings";
        }
        else
        {
          savedWifi = command.credentials;
          wifiError = "";
          beginWifiConnection();
        }
      }
      else if (command.type == WifiCommandType::Forget)
      {
        if (wifiPreferences.isKey("network") && !wifiPreferences.remove("network"))
        {
          wifiError = "Could not remove Wi-Fi settings";
        }
        else
        {
          savedWifi = WifiCredentials{};
          wifiConnecting = false;
          wifiConnected = false;
          wifiError = "";
          WiFi.disconnect();
          ensureAccessPoint();
        }
      }
    }

    if (wifiConnecting && WiFi.status() == WL_CONNECTED)
    {
      wifiConnecting = false;
      wifiConnected = true;
      wifiError = "";
      wifiConnectedAt = millis();
      Serial.print("Connected to Wi-Fi; panel: http://");
      Serial.println(WiFi.localIP());
    }
    else if (wifiConnecting &&
             static_cast<uint32_t>(millis() - wifiAttemptStarted) >= WIFI_CONNECT_TIMEOUT_MS)
    {
      wifiConnecting = false;
      WiFi.disconnect();
      wifiError = "Failed to connect to Wi-Fi";
      Serial.println(wifiError);
      ensureAccessPoint();
    }
    else if (wifiConnected && WiFi.status() != WL_CONNECTED)
    {
      wifiConnected = false;
      wifiError = "Wi-Fi connection lost";
      Serial.println(wifiError);
      ensureAccessPoint();
      lastWifiAttempt = millis();
    }
    else if (!wifiConnected && !wifiConnecting && savedWifi.ssid[0] &&
             static_cast<uint32_t>(millis() - lastWifiAttempt) >= WIFI_RETRY_INTERVAL_MS)
    {
      beginWifiConnection();
    }

    if (wifiConnected && accessPointStarted &&
        static_cast<uint32_t>(millis() - wifiConnectedAt) >= AP_HANDOVER_DELAY_MS)
    {
      if (WiFi.softAPdisconnect(true))
      {
        accessPointStarted = false;
      }
      else
      {
        wifiError = "Failed to disable fallback AP";
        Serial.println(wifiError);
        wifiConnectedAt = millis();
      }
    }

    updateWifiStatus();
  }

  bool queueWifiCommand(AsyncWebServerRequest *request, const WifiCommand &command)
  {
    portENTER_CRITICAL(&stateMux);
    if (wifiCommandPending)
    {
      portEXIT_CRITICAL(&stateMux);
      request->send(409, "application/json", "{\"error\":\"A Wi-Fi command is already pending\"}");
      return false;
    }
    pendingWifi = command;
    wifiCommandPending = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(202, "application/json", "{\"queued\":true}");
    return true;
  }

  void setupRoutes()
  {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    if (!SPIFFS.exists("/index.html")) {
      request->send(500, "text/plain", "Missing data/index.html");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html"); });
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    if (!SPIFFS.exists("/settings.html")) {
      request->send(404, "text/plain", "Missing data/settings.html");
      return;
    }
    request->send(SPIFFS, "/settings.html", "text/html"); });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    if (!SPIFFS.exists("/style.css")) {
      request->send(404, "text/plain", "Missing data/style.css");
      return;
    }
    request->send(SPIFFS, "/style.css", "text/css"); });
    server.on("/api/state", HTTP_GET, sendState);
    server.on("/api/poll", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    if (!request->hasParam("interval")) {
      request->send(400, "application/json", "{\"error\":\"Provide an interval in ms\"}");
      return;
    }
    const String raw = request->getParam("interval")->value();
    char *end = nullptr;
    const long interval = strtol(raw.c_str(), &end, 10);
    if (end == raw.c_str() || *end != '\0' ||
        interval < MIN_POLL_INTERVAL_MS || interval > MAX_POLL_INTERVAL_MS) {
      request->send(400, "application/json", "{\"error\":\"Interval must be 10-1000 ms\"}");
      return;
    }
    if (!settingsStorageReady) {
      request->send(503, "application/json", "{\"error\":\"Settings storage is unavailable\"}");
      return;
    }
    if (settingsPreferences.putUShort("poll-ms", static_cast<uint16_t>(interval)) !=
        sizeof(uint16_t)) {
      request->send(500, "application/json", "{\"error\":\"Could not save polling interval\"}");
      return;
    }
    portENTER_CRITICAL(&stateMux);
    pollIntervalMs = static_cast<uint16_t>(interval);
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "application/json", "{\"ok\":true}"); });
    server.on("/api/wifi", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "application/json", wifiStatusJson()); });
    server.on("/api/wifi", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    if (!request->hasParam("ssid", true) || !request->hasParam("password", true)) {
      request->send(400, "application/json", "{\"error\":\"Provide a network name and password\"}");
      return;
    }
    const String ssid = request->getParam("ssid", true)->value();
    const String password = request->getParam("password", true)->value();
    if (ssid.length() == 0 || ssid.length() > 32 ||
        (password.length() != 0 && (password.length() < 8 || password.length() > 63))) {
      request->send(400, "application/json",
                    "{\"error\":\"SSID: 1-32 bytes; password: 8-63 bytes or empty for an open network\"}");
      return;
    }
    WifiCommand command;
    command.type = WifiCommandType::Connect;
    command.queuedAt = millis();
    strlcpy(command.credentials.ssid, ssid.c_str(), sizeof(command.credentials.ssid));
    strlcpy(command.credentials.password, password.c_str(), sizeof(command.credentials.password));
    queueWifiCommand(request, command); });
    server.on("/api/wifi/forget", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    WifiCommand command;
    command.type = WifiCommandType::Forget;
    command.queuedAt = millis();
    queueWifiCommand(request, command); });

    server.on("/api/set", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    float voltage = 0.0f;
    float current = 0.0f;
    if (!parseNumber(request, "voltage", voltage) ||
        !parseNumber(request, "current", current)) {
      request->send(400, "application/json",
                    "{\"error\":\"Provide valid voltage and current values\"}");
      return;
    }

    const PsuState snapshot = copyState();
    if (!snapshot.connected) {
      request->send(409, "application/json",
                    "{\"error\":\"Power supply is not connected\"}");
      return;
    }
    if (!snapshot.remoteControl) {
      request->send(409, "application/json",
                    "{\"error\":\"Enable remote control first\"}");
      return;
    }
    if (voltage < 0.0f || current < 0.0f ||
        voltage > snapshot.maxVoltage || current > snapshot.maxCurrent) {
      request->send(400, "application/json",
                    "{\"error\":\"Value outside power supply range\"}");
      return;
    }

    PendingCommand command;
    command.type = CommandType::SetValues;
    command.voltage = voltage;
    command.current = current;
    queueCommand(request, command); });

    server.on("/api/output", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    PendingCommand command;
    command.type = CommandType::SetOutput;
    if (!parseEnabled(request, command.enabled)) {
      request->send(400, "application/json",
                    "{\"error\":\"enabled must be 0 or 1\"}");
      return;
    }
    if (!copyState().connected) {
      request->send(409, "application/json",
                    "{\"error\":\"Power supply is not connected\"}");
      return;
    }
    queueCommand(request, command); });

    server.on("/api/ocp", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    PendingCommand command;
    command.type = CommandType::SetOcp;
    if (!parseEnabled(request, command.enabled)) {
      request->send(400, "application/json",
                    "{\"error\":\"enabled must be 0 or 1\"}");
      return;
    }
    if (!copyState().connected) {
      request->send(409, "application/json",
                    "{\"error\":\"Power supply is not connected\"}");
      return;
    }
    queueCommand(request, command); });

    server.on("/api/remote", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    PendingCommand command;
    command.type = CommandType::SetRemote;
    if (!parseEnabled(request, command.enabled)) {
      request->send(400, "application/json",
                    "{\"error\":\"enabled must be 0 or 1\"}");
      return;
    }
    if (!copyState().connected) {
      request->send(409, "application/json",
                    "{\"error\":\"Power supply is not connected\"}");
      return;
    }
    queueCommand(request, command); });
  }

} // namespace

void setup()
{
  Serial.begin(19200);
  modbusSerial.begin(PSU_BAUD_RATE, SERIAL_8N1, PSU_RX_PIN, PSU_TX_PIN);

  if (!SPIFFS.begin())
  {
    Serial.println("Failed to mount SPIFFS");
  }

  settingsStorageReady = settingsPreferences.begin("wanptek", false);
  if (!settingsStorageReady)
  {
    Serial.println("Could not open polling settings storage");
  }
  else if (settingsPreferences.isKey("poll-ms"))
  {
    const uint16_t savedInterval = settingsPreferences.getUShort("poll-ms", 0);
    if (savedInterval >= MIN_POLL_INTERVAL_MS &&
        savedInterval <= MAX_POLL_INTERVAL_MS)
    {
      pollIntervalMs = savedInterval;
    }
    else
    {
      Serial.println("Invalid saved polling interval; using default");
      if (!settingsPreferences.remove("poll-ms"))
      {
        Serial.println("Could not remove invalid polling interval");
      }
    }
  }

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  wifiStorageReady = wifiPreferences.begin("wanptek-wifi", false);
  if (!wifiStorageReady)
  {
    wifiError = "Could not open Wi-Fi settings storage";
    Serial.println(wifiError);
  }
  else if (wifiPreferences.isKey("network"))
  {
    if (wifiPreferences.getBytesLength("network") != sizeof(savedWifi) ||
        wifiPreferences.getBytes("network", &savedWifi, sizeof(savedWifi)) != sizeof(savedWifi) ||
        !memchr(savedWifi.ssid, '\0', sizeof(savedWifi.ssid)) ||
        !memchr(savedWifi.password, '\0', sizeof(savedWifi.password)) ||
        !savedWifi.ssid[0])
    {
      savedWifi = WifiCredentials{};
      wifiError = "Invalid saved Wi-Fi settings";
      Serial.println(wifiError);
    }
  }
  if (savedWifi.ssid[0])
  {
    beginWifiConnection();
  }
  else
  {
    ensureAccessPoint();
  }
  setupRoutes();
  server.begin();
  updateWifiStatus();
  Serial.println("Web server started");
  printAccessPointStatus();
}

void loop()
{
  handleWifi();
  handleCommand();
  uint16_t interval;
  portENTER_CRITICAL(&stateMux);
  interval = pollIntervalMs;
  portEXIT_CRITICAL(&stateMux);
  if (static_cast<uint32_t>(millis() - lastPoll) >= interval)
  {
    lastPoll = millis();
    pollPsu();
  }
  if (verifyingSetpoints &&
      static_cast<uint32_t>(millis() - verifyStarted) >= SETPOINT_VERIFY_TIMEOUT_MS)
  {
    finishSetpointVerification(copyState(), false);
  }
  if (static_cast<uint32_t>(millis() - lastApStatus) >= AP_STATUS_INTERVAL_MS)
  {
    lastApStatus = millis();
    printAccessPointStatus();
  }
}
