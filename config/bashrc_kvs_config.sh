###############################################################################
# AWS KVS ENVIRONMENT CONFIGURATION FOR JETSON NANO
# Add these lines to ~/.bashrc on the Jetson Nano
###############################################################################

# ============================================================================
# GSTREAMER PLUGIN PATH
# ============================================================================
# Points to the compiled kvssink plugin directory
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/home/omar/amazon-kinesis-video-streams-producer-sdk-cpp/build

# ============================================================================
# LIBRARY PATH
# ============================================================================
# Ensures GStreamer can find libKinesisVideoProducer.so and dependencies
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/omar/amazon-kinesis-video-streams-producer-sdk-cpp/build

# ============================================================================
# AWS CREDENTIALS (Required for KVS streaming)
# ============================================================================
# These credentials are used by kvs_stream.launch and systemd service
export AWS_ACCESS_KEY_ID="AKIAQFLXNXMMILSXZBPW"
export AWS_SECRET_ACCESS_KEY="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"
export AWS_DEFAULT_REGION="us-east-1"

# ============================================================================
# VERIFICATION COMMANDS
# ============================================================================
# After adding to ~/.bashrc and running 'source ~/.bashrc', verify with:
#   gst-inspect-1.0 kvssink
#   echo $GST_PLUGIN_PATH
#   echo $LD_LIBRARY_PATH
