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

# Hardcoded AWS Configuration for RobotStream
echo "Configuring AWS credentials..."
AWS_ACCESS_KEY="AKIAQFLXNXMMILSXZBPW"
AWS_SECRET_KEY="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"
AWS_REGION="eu-west-1"
STREAM_NAME="RobotStream"
RETENTION_HOURS="24"

# Use aws configure set to programmatically write credentials
aws configure set aws_access_key_id "$AWS_ACCESS_KEY" --profile default
aws configure set aws_secret_access_key "$AWS_SECRET_KEY" --profile default
aws configure set region "$AWS_REGION" --profile default
aws configure set output json --profile default

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

# Verify AWS connection
echo "Verifying AWS credentials..."
if aws sts get-caller-identity &> /dev/null; then
    echo "✅ AWS connection verified!"
    aws sts get-caller-identity
else
    echo "❌ AWS connection failed!"
    exit 1
fi
echo ""

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
