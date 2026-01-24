#!/bin/bash
# AWS Configuration Script for Elderly Bot KVS Streaming
# Run this on your Jetson Nano

set -e

echo "================================================"
echo "AWS Configuration for Elderly Bot"
echo "================================================"
echo ""

# Check if AWS CLI is installed
if ! command -v aws &> /dev/null; then
    echo "❌ AWS CLI not found!"
    echo "Please run setup_camera_kvs.sh first to install dependencies"
    exit 1
fi

echo "✅ AWS CLI found"
echo ""

# Configure AWS credentials and settings interactively
echo "AWS Configuration Setup"
echo "========================"
echo ""
echo "Enter your AWS credentials:"
read -p "AWS Access Key ID: " AWS_ACCESS_KEY
read -sp "AWS Secret Access Key: " AWS_SECRET_KEY
echo ""
echo ""

echo "Enter AWS region (default: eu-north-1):"
read -p "AWS Region [eu-north-1]: " AWS_REGION
AWS_REGION=${AWS_REGION:-eu-north-1}
echo ""

echo "Enter Kinesis Video Stream name (default: elderly-bot-stream):"
read -p "Stream Name [elderly-bot-stream]: " STREAM_NAME
STREAM_NAME=${STREAM_NAME:-elderly-bot-stream}
echo ""

echo "Enter stream retention hours (default: 24):"
read -p "Retention Hours [24]: " RETENTION_HOURS
RETENTION_HOURS=${RETENTION_HOURS:-24}
echo ""

mkdir -p ~/.aws

cat > ~/.aws/credentials << EOF
[default]
aws_access_key_id = $AWS_ACCESS_KEY
aws_secret_access_key = $AWS_SECRET_KEY
EOF

cat > ~/.aws/config << EOF
[default]
region = $AWS_REGION
output = json
EOF

chmod 600 ~/.aws/credentials
chmod 644 ~/.aws/config

echo "✅ AWS credentials configured"
echo ""

# Test credentials
echo "Testing AWS connection..."
if aws sts get-caller-identity &> /dev/null; then
    echo "✅ AWS connection successful!"
    aws sts get-caller-identity
    echo ""
else
    echo "❌ AWS connection failed!"
    echo "Please check your credentials and internet connection"
    exit 1
fi

# Create KVS stream
echo "Creating Kinesis Video Stream: $STREAM_NAME"
echo ""

if aws kinesisvideo describe-stream --stream-name $STREAM_NAME --region $AWS_REGION &> /dev/null; then
    echo "ℹ️  Stream already exists"
    aws kinesisvideo describe-stream --stream-name $STREAM_NAME --region $AWS_REGION
else
    echo "Creating new stream..."
    aws kinesisvideo create-stream \
        --stream-name $STREAM_NAME \
        --data-retention-in-hours $RETENTION_HOURS \
        --region $AWS_REGION
    
    echo "✅ Stream created successfully!"
    echo ""
    
    # Wait for stream to be active
    echo "Waiting for stream to become active..."
    sleep 5
    aws kinesisvideo describe-stream --stream-name $STREAM_NAME --region $AWS_REGION
fi

echo ""
echo "================================================"
echo "✅ AWS Configuration Complete!"
echo "================================================"
echo ""
echo "Stream Details:"
echo "  Name: $STREAM_NAME"
echo "  Region: $AWS_REGION"
echo "  Retention: $RETENTION_HOURS hours"
echo ""
echo "Configuration saved to:"
echo "  ~/.aws/credentials"
echo "  ~/.aws/config"
echo ""
echo "Next steps:"
echo "  1. Test camera: roslaunch elderly_bot camera_streaming.launch enable_kvs:=false"
echo "  2. Test streaming: roslaunch elderly_bot camera_streaming.launch"
echo "  3. View stream: https://${AWS_REGION}.console.aws.amazon.com/kinesisvideo/home?region=${AWS_REGION}#/streams/$STREAM_NAME"
echo ""
