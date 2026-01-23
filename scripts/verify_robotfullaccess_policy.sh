#!/bin/bash
# Verify AWS IoT Policy Configuration
# =====================================
# Policy: RobotFullAccess
# Region: eu-north-1
# Account: 011524750104

echo "=========================================="
echo "AWS IoT Policy Verification"
echo "=========================================="
echo ""
echo "Your AWS Configuration:"
echo "  Region:     eu-north-1"
echo "  Account ID: 011524750104"
echo "  Policy ARN: arn:aws:iot:eu-north-1:011524750104:policy/RobotFullAccess"
echo ""

# Check if AWS CLI is installed
if ! command -v aws &> /dev/null; then
    echo "⚠️  AWS CLI not installed"
    echo ""
    echo "Install with:"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install -y awscli"
    echo "  aws configure"
    echo ""
    echo "Or check manually in AWS Console:"
    echo "  https://console.aws.amazon.com/iot"
    echo ""
    exit 1
fi

echo "=== Step 1: View Policy Document ==="
echo ""
echo "Fetching policy document..."
aws iot get-policy --policy-name RobotFullAccess --region eu-north-1 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Policy exists"
    echo ""
    echo "Check if the policy allows:"
    echo "  - iot:Connect for client 'elderly_bot_nano'"
    echo "  - iot:Publish for topic 'elderly_bot/*'"
    echo "  - iot:Subscribe for topicfilter 'elderly_bot/*'"
    echo "  - iot:Receive for topic 'elderly_bot/*'"
    echo ""
else
    echo "✗ Failed to fetch policy"
    echo "  Either AWS CLI is not configured or you don't have permissions"
    echo "  Check manually in AWS Console"
fi

echo ""
echo "=== Step 2: Check Policy Targets (Certificates) ==="
echo ""
echo "Listing certificates attached to this policy..."
aws iot list-policy-principals --policy-name RobotFullAccess --region eu-north-1 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ If you see certificate ARNs listed above, policy is attached"
    echo "✗ If empty, you need to attach the policy to your certificate"
else
    echo "✗ Failed to list policy targets"
    echo "  Check manually in AWS Console → Security → Policies → RobotFullAccess → Targets"
fi

echo ""
echo "=== Step 3: Verify Certificate is Active ==="
echo ""
echo "Manual check required:"
echo "  1. Go to: https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/certificatehub"
echo "  2. Find your certificate"
echo "  3. Status must be 'Active'"
echo ""

echo ""
echo "=== Step 4: Test Connection ==="
echo ""
echo "Run this to test connection with your policy:"
echo "  cd ~/catkin_ws/src/elderly_bot/scripts"
echo "  python aws_connection_test_port443.py"
echo ""

echo "=========================================="
echo "Required Policy Configuration"
echo "=========================================="
echo ""
echo "Your RobotFullAccess policy should contain:"
echo ""
cat << 'EOF'
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:client/elderly_bot_nano"
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish",
        "iot:Receive"
      ],
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topicfilter/elderly_bot/*"
    }
  ]
}
EOF
echo ""
echo "OR for testing (permissive):"
echo ""
cat << 'EOF'
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
EOF
echo ""
echo "=========================================="
echo ""
echo "To update policy in AWS Console:"
echo "  1. Go to: IoT Core → Security → Policies"
echo "  2. Click: RobotFullAccess"
echo "  3. Click: Edit active version"
echo "  4. Replace JSON with one of the examples above"
echo "  5. Click: Save as new version"
echo ""
