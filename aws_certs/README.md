# AWS IoT Core Certificates Directory

This directory stores the certificates required for AWS IoT Core authentication.

## ⚠️ IMPORTANT: Security Notice

**NEVER commit actual certificates to Git!**
- This folder is for **deployment only** on the Jetson Nano
- The `.gitignore` file should exclude `*.pem`, `*.crt`, `*.key` files
- Only README and placeholder files should be in version control

## Required Files

Place the following files in this directory (download from AWS IoT Console):

### 1. Root CA Certificate
**Filename**: `AmazonRootCA1.pem`
- Amazon's root certificate authority
- Download: https://www.amazontrust.com/repository/AmazonRootCA1.pem
- Or from AWS IoT Console during Thing creation
- **This file is public and safe to download**

### 2. Device Certificate
**Filename**: `device-certificate.pem.crt` (or rename your downloaded cert to this)
- Unique certificate for your robot/Thing
- Downloaded when you create a Thing in AWS IoT Console
- **Keep this file secure - it identifies your device**

### 3. Private Key
**Filename**: `device-private.pem.key` (or rename your downloaded key to this)
- Private key paired with the device certificate
- Downloaded when you create a Thing in AWS IoT Console
- **Keep this file VERY secure - never share or commit to Git**

## Setup Instructions

### On Jetson Nano:

1. **Create the certificates directory:**
   ```bash
   mkdir -p ~/aws_certs
   chmod 700 ~/aws_certs  # Secure permissions (owner only)
   ```

2. **Download Amazon Root CA:**
   ```bash
   cd ~/aws_certs
   wget https://www.amazontrust.com/repository/AmazonRootCA1.pem
   chmod 644 AmazonRootCA1.pem
   ```

3. **Copy your device certificate and private key:**
   
   From your development machine (Windows):
   ```bash
   # Copy the files you downloaded from AWS IoT Console
   scp /path/to/your-device-cert.pem.crt omar@192.168.1.29:~/aws_certs/device-certificate.pem.crt
   scp /path/to/your-private.pem.key omar@192.168.1.29:~/aws_certs/device-private.pem.key
   ```
   
   Or manually copy via USB drive, then on Jetson:
   ```bash
   cp /media/usb/your-device-cert.pem.crt ~/aws_certs/device-certificate.pem.crt
   cp /media/usb/your-private.pem.key ~/aws_certs/device-private.pem.key
   ```

4. **Set secure permissions:**
   ```bash
   cd ~/aws_certs
   chmod 644 device-certificate.pem.crt
   chmod 600 device-private.pem.key  # Private key: owner read-only
   ```

5. **Verify files exist:**
   ```bash
   ls -lh ~/aws_certs
   # Should show:
   # -rw-r--r-- AmazonRootCA1.pem
   # -rw-r--r-- device-certificate.pem.crt
   # -rw------- device-private.pem.key
   ```

6. **Update cloud_config.yaml:**
   The default config already points to `~/aws_certs/`, so no changes needed unless you use a different location.

## AWS IoT Core Setup (Step-by-Step)

If you haven't created your Thing in AWS IoT Console yet:

### Step 1: Create a Thing

1. Go to AWS IoT Console: https://console.aws.amazon.com/iot/
2. Navigate to: **Manage → All devices → Things**
3. Click **Create things**
4. Select **Create single thing**
5. Thing name: `robot_01` (or match your `client_id` in cloud_config.yaml)
6. Click **Next**

### Step 2: Configure Device Certificate

1. Select **Auto-generate a new certificate**
2. Click **Next**

### Step 3: Attach Policy

1. If you don't have a policy yet, click **Create policy**
   
   Policy name: `RobotIoTPolicy`
   
   Policy document (JSON):
   ```json
   {
     "Version": "2012-10-17",
     "Statement": [
       {
         "Effect": "Allow",
         "Action": "iot:Connect",
         "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:client/robot_*"
       },
       {
         "Effect": "Allow",
         "Action": "iot:Publish",
         "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/robot/*"
       },
       {
         "Effect": "Allow",
         "Action": "iot:Subscribe",
         "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topicfilter/robot/*"
       },
       {
         "Effect": "Allow",
         "Action": "iot:Receive",
         "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/robot/*"
       }
     ]
   }
   ```
   
   Replace `REGION` with your AWS region (e.g., `us-east-1`)
   Replace `ACCOUNT_ID` with your AWS account ID

2. Attach the policy to your certificate
3. Click **Create thing**

### Step 4: Download Certificates

**CRITICAL: Download these files now - you can't download them again!**

1. **Device certificate** → Save as `device-certificate.pem.crt`
2. **Private key** → Save as `device-private.pem.key`
3. **Amazon Root CA 1** → Save as `AmazonRootCA1.pem`

Store these files securely on your development machine, then transfer to Jetson as described above.

### Step 5: Get Your AWS IoT Endpoint

1. In AWS IoT Console, go to **Settings**
2. Copy your **Endpoint** (looks like: `a1b2c3d4e5f6g7-ats.iot.us-east-1.amazonaws.com`)
3. Paste this into `config/cloud_config.yaml` → `aws_endpoint` parameter

### Step 6: Test Connection

On Jetson Nano:
```bash
# Test MQTT connection (install mosquitto-clients if needed)
mosquitto_pub --cafile ~/aws_certs/AmazonRootCA1.pem \
              --cert ~/aws_certs/device-certificate.pem.crt \
              --key ~/aws_certs/device-private.pem.key \
              -h YOUR_ENDPOINT.iot.REGION.amazonaws.com \
              -p 8883 \
              -q 1 \
              -t robot/test \
              -i robot_01 \
              --tls-version tlsv1.2 \
              -m "Hello from robot" \
              -d

# If successful, launch ROS node
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

## Directory Structure

```
aws_certs/
├── README.md                          # This file (safe to commit)
├── .gitignore                         # Excludes certificates (safe to commit)
├── PLACEHOLDER_AmazonRootCA1.pem     # Example structure (safe to commit)
├── PLACEHOLDER_device-cert.pem.crt   # Example structure (safe to commit)
└── PLACEHOLDER_device-key.pem.key    # Example structure (safe to commit)

# After setup on Jetson, you'll have:
├── AmazonRootCA1.pem                 # NEVER COMMIT
├── device-certificate.pem.crt        # NEVER COMMIT
└── device-private.pem.key            # NEVER COMMIT
```

## Troubleshooting

### Connection Refused
- Check endpoint URL in cloud_config.yaml
- Verify certificates are in correct location
- Ensure files have correct permissions

### Certificate Verification Failed
- Verify you downloaded the correct Root CA (AmazonRootCA1.pem)
- Check certificate expiration date
- Ensure device certificate is activated in AWS IoT Console

### Policy Denied
- Review IoT policy in AWS Console
- Ensure policy is attached to your certificate
- Check resource ARNs match your region/account

### File Permission Errors
```bash
# Fix permissions
chmod 700 ~/aws_certs
chmod 644 ~/aws_certs/AmazonRootCA1.pem
chmod 644 ~/aws_certs/device-certificate.pem.crt
chmod 600 ~/aws_certs/device-private.pem.key
```

## Security Best Practices

1. ✅ **Use unique credentials per device**
2. ✅ **Never commit certificates to version control**
3. ✅ **Set strict file permissions (600 for private keys)**
4. ✅ **Rotate certificates periodically**
5. ✅ **Use restrictive IoT policies (principle of least privilege)**
6. ✅ **Monitor CloudWatch logs for unauthorized access attempts**
7. ✅ **Revoke certificates immediately if device is compromised**

## Quick Reference

| File | Purpose | Permissions | Source |
|------|---------|-------------|--------|
| `AmazonRootCA1.pem` | Verify AWS server | 644 | https://amazontrust.com |
| `device-certificate.pem.crt` | Identify device | 644 | AWS IoT Console |
| `device-private.pem.key` | Authenticate device | **600** | AWS IoT Console |

---

**Last Updated**: January 22, 2026
