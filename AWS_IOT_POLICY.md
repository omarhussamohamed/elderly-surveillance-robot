# AWS IoT Policy Configuration Guide
# ====================================

## Required AWS IoT Policy for Elderly Bot

When you create a Thing in AWS IoT Core, you must attach a Policy to the certificate.
This policy controls what MQTT operations your device can perform.

### Policy JSON Template

Copy this JSON and create a new policy in AWS IoT Console:

**AWS IoT Console → Security → Policies → Create**

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": [
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:client/elderly_bot_nano",
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:client/elderly_bot_nano_test",
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:client/elderly_bot_01_test",
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:client/elderly_bot_nano_test_443"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish",
        "iot:Receive"
      ],
      "Resource": [
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:topic/elderly_bot/*",
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:topic/robot/*"
      ]
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": [
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:topicfilter/elderly_bot/*",
        "arn:aws:iot:eu-north-1:YOUR_ACCOUNT_ID:topicfilter/robot/*"
      ]
    }
  ]
}
```

### How to Get Your Account ID

**Option 1: From AWS Console**
- Click your username (top right) → Copy Account ID

**Option 2: From AWS CLI**
```bash
aws sts get-caller-identity --query Account --output text
```

### Complete Example (with actual Account ID)

Replace `123456789012` with your actual AWS Account ID:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": [
        "arn:aws:iot:eu-north-1:123456789012:client/elderly_bot_nano",
        "arn:aws:iot:eu-north-1:123456789012:client/elderly_bot_nano_test",
        "arn:aws:iot:eu-north-1:123456789012:client/elderly_bot_01_test",
        "arn:aws:iot:eu-north-1:123456789012:client/elderly_bot_nano_test_443"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish",
        "iot:Receive"
      ],
      "Resource": [
        "arn:aws:iot:eu-north-1:123456789012:topic/elderly_bot/*",
        "arn:aws:iot:eu-north-1:123456789012:topic/robot/*"
      ]
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": [
        "arn:aws:iot:eu-north-1:123456789012:topicfilter/elderly_bot/*",
        "arn:aws:iot:eu-north-1:123456789012:topicfilter/robot/*"
      ]
    }
  ]
}
```

### Policy Explanation

**Statement 1: Connect Permission**
- Allows your device to connect to AWS IoT Core
- Must match your `client_id` parameter
- Multiple client IDs listed for testing variants

**Statement 2: Publish/Receive Permission**
- `iot:Publish` - Send messages TO cloud
- `iot:Receive` - Receive messages FROM cloud
- `elderly_bot/*` - All topics under elderly_bot/
- `robot/*` - Legacy topic tree (for backward compatibility)

**Statement 3: Subscribe Permission**
- `iot:Subscribe` - Listen to MQTT topics
- Must use `topicfilter` (not `topic`)
- Wildcard `/*` allows all sub-topics

### Testing Your Policy

After creating and attaching the policy:

1. **Check in AWS Console:**
   - AWS IoT Core → Security → Certificates
   - Click your certificate
   - Verify policy is attached

2. **Test connection:**
   ```bash
   python aws_connection_test.py
   ```

3. **Monitor in AWS:**
   - AWS IoT Core → Monitor → Logs
   - Look for connection attempts and errors

### Common Policy Errors

| Error | Cause | Fix |
|-------|-------|-----|
| "Not authorized to connect" | client_id not in policy | Add your client_id to Connect resource |
| "Not authorized to publish" | Topic not allowed | Add topic to Publish resource |
| "Connection closed immediately" | Policy not attached | Attach policy to certificate |

### Verification Commands

```bash
# Check if certificate is activated
aws iot describe-certificate --certificate-id YOUR_CERT_ID

# List policies attached to certificate
aws iot list-attached-policies --target YOUR_CERT_ARN

# Get policy document
aws iot get-policy --policy-name YOUR_POLICY_NAME
```

### Wildcard Policy (For Testing Only)

**⚠️ NOT RECOMMENDED FOR PRODUCTION**

Use this only for initial testing, then restrict to specific topics:

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

**Quick Setup Checklist:**

- [ ] Get AWS Account ID
- [ ] Create policy with above JSON (replace YOUR_ACCOUNT_ID)
- [ ] Attach policy to your certificate
- [ ] Verify certificate is ACTIVATED
- [ ] Test connection: `python aws_connection_test.py`
- [ ] Check AWS IoT Console → Monitor → Logs

---

**Region Note:** 
This example uses `eu-north-1`. Replace with your region if different.
