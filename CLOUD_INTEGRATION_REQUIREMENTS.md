# Cloud Integration Requirements for Elderly Bot
**Project:** Elderly Care Robot - AWS IoT Core Integration  
**Date:** January 23, 2026  
**Robot Platform:** Jetson Nano (Ubuntu 18.04, ROS Melodic)  
**AWS Region:** eu-north-1  
**AWS Account:** 011524750104

---

## 1. AWS IoT Core Configuration Requirements

### 1.1 Thing Creation
Please create an IoT Thing with the following specifications:

- **Thing Name:** `robot_nano` (exact name required)
- **Thing Type:** (Optional) `robot_device`
- **Region:** `eu-north-1`

### 1.2 Device Certificate
- Generate a new device certificate using **Auto-generate certificate** option
- **Certificate Status:** Set to **ACTIVE**
- Attach the certificate to Thing: `robot_nano`
- Attach the security policy (see Section 2) to this certificate

### 1.3 Endpoint Configuration
- **Endpoint Type:** ATS (Amazon Trust Services) Endpoint
- Please provide the full endpoint URL from: AWS IoT Core → Settings → Device data endpoint
- Expected format: `<unique-id>-ats.iot.eu-north-1.amazonaws.com`

---

## 2. Security Policy Requirements

### 2.1 Policy Name
**Policy Name:** `ElderlyBot_LeastPrivilege_Policy`

### 2.2 Policy Document (Principle of Least Privilege)

Please create an IoT Policy with the following JSON configuration:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:client/robot_nano",
      "Condition": {
        "Bool": {
          "iot:Connection.Thing.IsAttached": "true"
        }
      }
    },
    {
      "Effect": "Allow",
      "Action": "iot:Publish",
      "Resource": [
        "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/telemetry",
        "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/alerts"
      ]
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topicfilter/elderly_bot/commands"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Receive",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/commands"
    }
  ]
}
```

### 2.3 Policy Explanation

| Statement | Purpose |
|-----------|---------|
| **iot:Connect** | Allows only client `robot_nano` to connect, with condition that certificate is attached to a Thing |
| **iot:Publish** | Robot can publish sensor data to `telemetry` and urgent notifications to `alerts` |
| **iot:Subscribe** | Robot can subscribe to command topic to receive instructions from cloud |
| **iot:Receive** | Robot can receive messages on the subscribed command topic |

### 2.4 Policy Attachment
- Attach `ElderlyBot_LeastPrivilege_Policy` to the device certificate
- Verify attachment in: AWS IoT Core → Security → Certificates → [Certificate] → Policies tab

---

## 3. Network Connectivity Requirements

### 3.1 Network Constraints
The robot operates on a **restricted network** with the following limitations:
- Standard MQTT port **8883 is blocked** by firewall/ISP
- Only **HTTPS port 443** is allowed for outbound connections

### 3.2 Required Configuration
- **Protocol:** MQTT over TLS
- **Port:** 443 (HTTPS)
- **ALPN Required:** Yes
  - ALPN (Application Layer Protocol Negotiation) is required for MQTT over port 443
  - AWS IoT Core automatically handles ALPN when clients connect on port 443
  - No special server-side configuration needed

### 3.3 Connection Details
```
Protocol:  MQTT 3.1.1
Transport: TLS 1.2
Port:      443
ALPN:      x-amzn-mqtt-ca (handled automatically by AWS SDK)
```

---

## 4. MQTT Topics Structure

### 4.1 Published Topics (Robot → Cloud)

**Topic:** `elderly_bot/telemetry`
- **Purpose:** Periodic sensor data and system health metrics
- **Frequency:** 1 Hz (once per second)
- **Message Format:** JSON
- **Example Payload:**
```json
{
  "timestamp": 1737648000.123,
  "robot_id": "robot_nano",
  "telemetry": {
    "temperature": 45.2,
    "power": 12.5,
    "gas_detected": false
  }
}
```

**Topic:** `elderly_bot/alerts`
- **Purpose:** Urgent notifications and safety alerts
- **Frequency:** Event-driven (triggered on detection)
- **Message Format:** JSON
- **Example Payload:**
```json
{
  "timestamp": 1737648000.456,
  "robot_id": "robot_nano",
  "alert_type": "gas_detected",
  "value": true,
  "priority": "high"
}
```

### 4.2 Subscribed Topics (Cloud → Robot)

**Topic:** `elderly_bot/commands`
- **Purpose:** Receive commands from cloud/dashboard
- **Message Format:** JSON
- **Example Payload (Buzzer Control):**
```json
{
  "command": "buzzer",
  "value": 2000
}
```

---

## 5. Certificate and Credentials Deliverables

Please provide the following files/information after setup:

### 5.1 Required Files

| File | Description | Format | Security |
|------|-------------|--------|----------|
| **Root CA Certificate** | Amazon Root CA 1 | `.pem` | Public - Safe to share |
| **Device Certificate** | Thing-specific certificate | `.pem.crt` | Public - Safe to share |
| **Private Key** | Device private key | `.pem.key` | **CONFIDENTIAL** - Secure transfer required |

### 5.2 Required Configuration Information

- **AWS IoT Endpoint URL** (ATS endpoint)
  - Example: `a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com`
- **Certificate ID** (for verification)
- **Policy ARN** (for documentation)

### 5.3 Secure Transfer Method

For the **Private Key** file, please use one of the following secure methods:
- AWS Secrets Manager (preferred)
- Encrypted ZIP with separate password delivery
- Secure file sharing service with expiration
- Direct transfer via SSH/SCP if on same secure network

**DO NOT send the private key via:**
- Unencrypted email
- Slack/Teams chat
- SMS/WhatsApp

---

## 6. Verification Checklist

After setup, please verify the following:

### 6.1 AWS Console Verification

- [ ] Thing `robot_nano` exists in region `eu-north-1`
- [ ] Device certificate is **ACTIVE** (green status)
- [ ] Policy `ElderlyBot_LeastPrivilege_Policy` is created
- [ ] Policy is **attached** to the device certificate
- [ ] Certificate is **attached** to Thing `robot_nano`
- [ ] ATS endpoint is documented and provided

### 6.2 Policy Testing (Optional)

If possible, verify the policy allows the following operations:
```
✓ iot:Connect with client_id = "robot_nano"
✓ iot:Publish to "elderly_bot/telemetry"
✓ iot:Publish to "elderly_bot/alerts"
✓ iot:Subscribe to "elderly_bot/commands"
✗ iot:Publish to "other_robot/data" (should be DENIED)
✗ iot:Connect with client_id = "different_client" (should be DENIED)
```

---

## 7. Additional Information

### 7.1 Robot-Side Configuration

Once credentials are received, the robot will be configured with:
- Client ID: `robot_nano` (matches Thing name)
- MQTT Protocol: v3.1.1
- Keepalive: 30 seconds
- Auto-reconnect: Enabled (exponential backoff)
- QoS: 1 (at least once delivery)

### 7.2 Testing Plan

After credential integration:
1. Standalone connection test (no ROS dependencies)
2. Certificate validation and TLS handshake verification
3. MQTT publish/subscribe functional test
4. Integration with ROS navigation system
5. End-to-end telemetry and command flow test

### 7.3 Monitoring Requirements

Please enable the following for troubleshooting:
- **CloudWatch Logs:** IoT Core → Settings → Logs → Level: Info
- **Metrics:** Connection attempts, publish/subscribe counts
- **Retain logging** for at least 7 days during initial deployment

---

## 8. Contact Information

**Technical Contact (Robot Side):**
- Name: Omar
- Platform: Jetson Nano
- ROS Version: Melodic (Python 2.7)

**Expected Timeline:**
- Credentials needed by: [SPECIFY DATE]
- Testing window: [SPECIFY DATE RANGE]

---

## 9. Troubleshooting Guide (For Reference)

### Common Issues and Solutions

| Issue | Probable Cause | Solution |
|-------|----------------|----------|
| Immediate disconnect | Policy not attached to certificate | Verify policy attachment in AWS Console |
| "Not authorized to connect" | client_id mismatch or missing in policy | Verify Resource ARN contains `robot_nano` |
| "Not authorized to publish" | Topic not in policy Resource list | Add topic ARN to Publish statement |
| Connection timeout | Port 443 blocked or endpoint incorrect | Verify endpoint URL, test port 443 with `nc -zv <endpoint> 443` |
| Certificate error | Certificate not ACTIVE or expired | Activate certificate in AWS Console |

---

## 10. Success Criteria

The AWS IoT Core setup will be considered complete when:

1. ✅ Robot can connect to AWS IoT Core using client_id `robot_nano` on port 443
2. ✅ Robot can successfully publish telemetry messages to `elderly_bot/telemetry`
3. ✅ Robot can successfully publish alerts to `elderly_bot/alerts`
4. ✅ Robot can receive and process commands from `elderly_bot/commands`
5. ✅ Connection remains stable with automatic reconnection on network interruption
6. ✅ CloudWatch logs show successful connections and message flow

---

**Document Version:** 1.0  
**Last Updated:** January 23, 2026  
**Prepared By:** Elderly Bot Development Team

---

## Appendix A: Quick Reference

### AWS Console Links (eu-north-1)

- **IoT Things:** https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/thinghub
- **Certificates:** https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/certificatehub
- **Policies:** https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/policyhub
- **MQTT Test Client:** https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/test
- **Settings (Endpoint):** https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/settings

### Root CA Download

Amazon Root CA 1 (public):
```
https://www.amazontrust.com/repository/AmazonRootCA1.pem
```

### Policy Summary (One-Line)

"Allow robot_nano to connect via port 443, publish to telemetry/alerts, and subscribe to commands topic with Thing attachment condition."
