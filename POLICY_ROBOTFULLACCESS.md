# AWS IoT Policy Configuration for Account 011524750104
# ======================================================

## Your Specific Configuration

**Region:** eu-north-1  
**Account ID:** 011524750104  
**Policy Name:** RobotFullAccess  
**Policy ARN:** `arn:aws:iot:eu-north-1:011524750104:policy/RobotFullAccess`

---

## Required Policy JSON

Copy this into AWS Console → IoT Core → Security → Policies → RobotFullAccess → Edit:

### Option 1: Specific Permissions (Recommended)

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": [
        "arn:aws:iot:eu-north-1:011524750104:client/elderly_bot_nano",
        "arn:aws:iot:eu-north-1:011524750104:client/elderly_bot_nano_test",
        "arn:aws:iot:eu-north-1:011524750104:client/elderly_bot_nano_test_443"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish",
        "iot:Receive"
      ],
      "Resource": [
        "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/*",
        "arn:aws:iot:eu-north-1:011524750104:topic/robot/*"
      ]
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": [
        "arn:aws:iot:eu-north-1:011524750104:topicfilter/elderly_bot/*",
        "arn:aws:iot:eu-north-1:011524750104:topicfilter/robot/*"
      ]
    }
  ]
}
```

### Option 2: Full Access (Testing Only)

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

---

## How to Update Policy in AWS Console

1. **Navigate to Policy:**
   ```
   AWS Console → IoT Core → Security → Policies → RobotFullAccess
   ```

2. **Edit Policy:**
   - Click **"Edit active version"** or **"Edit policy document"**
   - Replace entire JSON with one of the options above
   - Click **"Save as new version"**

3. **Direct Link:**
   ```
   https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/policyhub/RobotFullAccess
   ```

---

## Verify Policy is Attached to Certificate

### Method 1: From Certificate View
1. Go to: IoT Core → Security → Certificates
2. Click on your certificate
3. Go to **"Policies"** tab
4. Verify `RobotFullAccess` is listed
5. If not, click **"Attach policy"** → Select `RobotFullAccess`

### Method 2: From Policy View
1. Go to: IoT Core → Security → Policies → RobotFullAccess
2. Go to **"Targets"** tab
3. Your certificate should be listed
4. If not, click **"Attach"** → Select your certificate

---

## Verification Script

Run on your Jetson Nano:

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x verify_robotfullaccess_policy.sh
./verify_robotfullaccess_policy.sh
```

This will check:
- Policy document contents
- Which certificates have this policy attached
- Provide direct links to AWS Console

---

## Test Connection After Policy Update

```bash
# Test with port 443
cd ~/catkin_ws/src/elderly_bot/scripts
python aws_connection_test_port443.py
```

Expected output:
```
✅ CONNECTION SUCCESSFUL on Port 443!
```

---

## Common Policy Issues

| Issue | Cause | Fix |
|-------|-------|-----|
| "Not authorized to connect" | client_id not in Connect Resource | Add `client/elderly_bot_nano` to policy |
| "Not authorized to publish" | topic not in Publish Resource | Add `topic/elderly_bot/*` to policy |
| Immediate disconnect | Policy not attached to cert | Attach RobotFullAccess to certificate |
| Policy exists but not working | Old version active | Save as new version |

---

## AWS CLI Commands (Optional)

If you have AWS CLI configured on Jetson:

```bash
# View policy document
aws iot get-policy --policy-name RobotFullAccess --region eu-north-1

# List certificates with this policy
aws iot list-policy-principals --policy-name RobotFullAccess --region eu-north-1

# Update policy (from file)
aws iot create-policy-version \
  --policy-name RobotFullAccess \
  --policy-document file://new_policy.json \
  --set-as-default \
  --region eu-north-1
```

---

## Next Steps

1. ✅ Update RobotFullAccess policy with JSON above
2. ✅ Verify policy is attached to certificate
3. ✅ Verify certificate status is "Active"
4. ✅ Run: `python aws_connection_test_port443.py`
5. ✅ Check AWS IoT Monitor → Logs for connection attempts
6. ✅ Launch ROS: `roslaunch elderly_bot bringup.launch enable_cloud:=true`

---

## If Still Having Issues

Enable AWS IoT Core logging:

1. **Go to:** IoT Core → Settings → Logs
2. **Set level:** Debug (All)
3. **Create role** if needed (AWS will auto-create)
4. **Try connecting** again
5. **View logs:** CloudWatch → Log groups → AWSIotLogsV2

This will show exactly why AWS is rejecting the connection.
