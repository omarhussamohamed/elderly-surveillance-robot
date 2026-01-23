# AWS IoT Console Configuration Checklist
# ==========================================

## 🚨 IMMEDIATE ISSUE: YAML Syntax Error

Your launch file is failing because of a YAML parsing error. Run this command on your Jetson Nano:

```bash
cd ~/catkin_ws/src/elderly_bot/config
python -c "import yaml; yaml.safe_load(open('cloud_config.yaml'))"
```

**If you get an error:** The YAML file has syntax issues (possibly invisible characters or encoding problems).

**Fix:** Re-save the file with proper UTF-8 encoding:
```bash
# Backup current file
cp cloud_config.yaml cloud_config.yaml.backup

# Remove BOM and fix encoding
sed '1s/^\xEF\xBB\xBF//' cloud_config.yaml.backup > cloud_config.yaml
```

---

## 1. AWS IoT Policy Template (Permissive for Testing)

Copy this exact JSON into AWS Console → IoT Core → Security → Policies:

**Policy Name:** `elderly_bot_test_policy`

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:*",
      "Resource": "*"
    }
  ]
}
```

### ⚠️ WARNING
This policy allows ALL IoT operations on ALL resources. Use ONLY for testing.

### Production Policy (More Restrictive)

Replace `YOUR_ACCOUNT_ID` and `YOUR_REGION` with your actual values:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": [
        "arn:aws:iot:YOUR_REGION:YOUR_ACCOUNT_ID:client/elderly_bot_nano",
        "arn:aws:iot:YOUR_REGION:YOUR_ACCOUNT_ID:client/elderly_bot_*"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish",
        "iot:Receive"
      ],
      "Resource": "arn:aws:iot:YOUR_REGION:YOUR_ACCOUNT_ID:topic/elderly_bot/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:YOUR_REGION:YOUR_ACCOUNT_ID:topicfilter/elderly_bot/*"
    }
  ]
}
```

**To find YOUR_ACCOUNT_ID:**
- AWS Console → Click your username (top right)
- Copy the 12-digit Account ID

**YOUR_REGION:** `eu-north-1` (from your endpoint)

---

## 2. AWS Console 3-Step Verification Checklist

### ✅ Step 1: Certificate Status

**Location:** AWS IoT Core → Security → Certificates

**Check:**
- [ ] Certificate exists in list
- [ ] Status column shows **"Active"** (green indicator)
- [ ] If status is "Inactive" or "Revoked":
  - Click on the certificate
  - Click **"Activate"** button (top right)

### ✅ Step 2: Policy Attachment

**Location:** AWS IoT Core → Security → Certificates → [Your Certificate]

**Check:**
- [ ] Click on your certificate
- [ ] Go to **"Policies"** tab
- [ ] At least ONE policy is listed (e.g., `elderly_bot_test_policy`)
- [ ] If NO policies shown:
  - Click **"Attach policy"**
  - Select `elderly_bot_test_policy`
  - Click **"Attach"**

**Alternative:** Go to Security → Policies → [Your Policy] → **"Targets"** tab → Verify your certificate is listed

### ✅ Step 3: Thing Name Match

**Location:** AWS IoT Core → Manage → Things

**Check:**
- [ ] Search for Thing named: `elderly_bot_nano`
- [ ] If Thing exists with THIS EXACT NAME → ✅ Client ID matches
- [ ] If Thing has DIFFERENT name (e.g., `ElderlyBotThing`):
  - Either: Rename Thing to `elderly_bot_nano` in AWS
  - Or: Update `client_id` in your `cloud_config.yaml` to match Thing name
- [ ] If NO Thing exists:
  - Create new Thing: **"Create things"** → Single thing
  - Thing name: `elderly_bot_nano`
  - Attach your existing certificate to this Thing

---

## 3. Client ID Consistency Explanation

### ❓ Does client_id Need to Match Thing Name?

**Short Answer:** Not strictly required, but **strongly recommended** for organization.

### How AWS IoT Uses client_id

1. **Connection Identifier:** The `client_id` is what AWS IoT Core sees when your device connects
2. **Policy Matching:** Your policy uses `client_id` to control who can connect:
   ```json
   "Resource": "arn:aws:iot:REGION:ACCOUNT:client/elderly_bot_nano"
   ```
3. **Thing Association:** The Thing name is for device management/shadow/jobs

### Your Current Configuration

**File:** `cloud_config.yaml`
```yaml
client_id: "elderly_bot_nano"
```

**What This Means:**
- Your device connects to AWS with identifier `elderly_bot_nano`
- Your policy MUST allow this client_id to connect
- Thing name is separate (can be different, but matching is cleaner)

### Configuration Options

#### Option A: Match Everything (Recommended)
```
Thing Name:  elderly_bot_nano
Client ID:   elderly_bot_nano
Policy:      Allows client/elderly_bot_nano
```

#### Option B: Different Names (Works, but confusing)
```
Thing Name:  ElderlyBotDevice
Client ID:   elderly_bot_nano
Policy:      Allows client/elderly_bot_nano
```

### How to Verify Your Setup

**Check your current client_id:**
```bash
grep "client_id:" ~/catkin_ws/src/elderly_bot/config/cloud_config.yaml
```

**Output:** `client_id: "elderly_bot_nano"`

**Check Thing name in AWS Console:**
- IoT Core → Manage → Things
- Look for Thing with name `elderly_bot_nano`

**If Thing name is different, you have 2 options:**

#### Option 1: Change config to match AWS Thing
```bash
nano ~/catkin_ws/src/elderly_bot/config/cloud_config.yaml
```

Change line:
```yaml
client_id: "YOUR_AWS_THING_NAME"  # Replace with actual Thing name from AWS
```

#### Option 2: Change AWS Thing name to match config
- AWS Console → IoT Core → Manage → Things
- Select your Thing → Actions → Update
- Change name to `elderly_bot_nano`

---

## Quick Diagnostic Commands

### Test YAML Syntax
```bash
python -c "import yaml; print(yaml.safe_load(open('config/cloud_config.yaml')))"
```

### Check Client ID in Config
```bash
grep -A1 "client_id:" config/cloud_config.yaml
```

### Test AWS Connection
```bash
cd scripts
python aws_connection_test_port443.py
```

### View AWS Connection Logs
```bash
tail -f ~/.ros/log/latest/cloud_bridge_node*.log
```

---

## Common "Disconnect Event" Causes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Immediate disconnect | Policy doesn't allow client_id | Use permissive policy (iot:* on *) |
| Disconnect after 1-2 sec | Certificate inactive | Activate certificate in AWS Console |
| Disconnect on publish | Policy doesn't allow topics | Add iot:Publish on topic ARN |
| "Not authorized" | Policy not attached to cert | Attach policy to certificate |

---

## Verification Script

Run this interactive script:
```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x verify_aws_console.sh
./verify_aws_console.sh
```

This will guide you through all AWS Console checks step-by-step.

---

## After Fixing AWS Console

1. **Re-test connection:**
   ```bash
   python scripts/aws_connection_test_port443.py
   ```

2. **Launch ROS:**
   ```bash
   roslaunch elderly_bot bringup.launch enable_cloud:=true
   ```

3. **Monitor AWS Logs:**
   - AWS Console → IoT Core → Monitor → Logs
   - Enable CloudWatch logs if not already enabled

4. **Check ROS logs:**
   ```bash
   rostopic echo /rosout | grep cloud_bridge
   ```

---

## Still Having Issues?

Enable verbose AWS SDK logging:

```bash
export AWSIOT_LOG_LEVEL=DEBUG
python scripts/aws_connection_test_port443.py
```

This will show detailed TLS handshake and MQTT protocol messages.
