#!/bin/bash
# AWS IoT Console Verification Script
# =====================================
# Run this script to get step-by-step instructions for verifying
# your AWS IoT Core configuration.

echo "=========================================="
echo "AWS IoT Console Verification Guide"
echo "=========================================="
echo ""
echo "Follow these steps in the AWS Console to diagnose connection issues."
echo ""

# === AWS Policy Configuration ===
echo "=== STEP 1: Verify AWS IoT Policy ==="
echo ""
echo "1. Open AWS Console → IoT Core → Security → Policies"
echo "2. Find your policy (or create a new one named 'elderly_bot_policy')"
echo "3. Click on the policy name"
echo "4. Click 'Edit policy document' or 'Edit active version'"
echo "5. Replace the entire JSON with this PERMISSIVE TEST POLICY:"
echo ""
echo "─────────────────────────────────────────"
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
echo "─────────────────────────────────────────"
echo ""
echo "⚠️  WARNING: This policy allows ALL IoT actions on ALL resources."
echo "    Use ONLY for testing. Restrict permissions in production."
echo ""
echo "6. Click 'Save as new version'"
echo ""
read -p "Press Enter when policy is updated..."
echo ""

# === Certificate Activation ===
echo "=== STEP 2: Verify Certificate Status ==="
echo ""
echo "1. Open AWS Console → IoT Core → Security → Certificates"
echo "2. Find your certificate in the list"
echo "3. Check the 'Status' column:"
echo "   ✓ Must show 'Active' (green)"
echo "   ✗ If 'Inactive' or 'Revoked', click the certificate and click 'Activate'"
echo ""
echo "4. Click on your certificate"
echo "5. Go to the 'Policies' tab"
echo "6. Verify that 'elderly_bot_policy' (or your policy name) is listed"
echo "7. If NO policies are attached:"
echo "   - Click 'Attach policy'"
echo "   - Select your policy"
echo "   - Click 'Attach'"
echo ""
read -p "Press Enter when certificate is verified..."
echo ""

# === Thing Configuration ===
echo "=== STEP 3: Verify Thing Name and Client ID ==="
echo ""
echo "Your current configuration:"
echo "  Client ID: elderly_bot_nano"
echo ""
echo "1. Open AWS Console → IoT Core → Manage → Things"
echo "2. Check if you have a Thing named 'elderly_bot_nano'"
echo ""
echo "Options:"
echo ""
echo "  A) Thing exists with name 'elderly_bot_nano':"
echo "     ✓ Configuration is correct"
echo "     ✓ Client ID matches Thing name"
echo ""
echo "  B) Thing exists with DIFFERENT name (e.g., 'ElderlyBot_Thing'):"
echo "     → Update cloud_config.yaml:"
echo "       client_id: \"ElderlyBot_Thing\""
echo ""
echo "  C) No Thing exists:"
echo "     → Create a new Thing:"
echo "       - IoT Core → Manage → Things → Create"
echo "       - Thing name: elderly_bot_nano"
echo "       - Attach your certificate to this Thing"
echo ""
read -p "Press Enter when Thing configuration is verified..."
echo ""

# === Connection Test Command ===
echo "=== STEP 4: Test Connection ==="
echo ""
echo "After verifying all settings above, test the connection:"
echo ""
echo "  cd ~/catkin_ws/src/elderly_bot/scripts"
echo "  python aws_connection_test_port443.py"
echo ""
echo "Expected output: ✅ CONNECTION SUCCESSFUL"
echo ""
echo "If still failing, check AWS IoT Monitor:"
echo "  AWS Console → IoT Core → Monitor → Logs"
echo "  Look for connection attempts and rejection reasons"
echo ""
echo "=========================================="
echo "Verification Complete"
echo "=========================================="
echo ""
echo "Summary of required settings:"
echo "  ✓ Policy: Allow iot:* on Resource: *"
echo "  ✓ Certificate: Status = Active"
echo "  ✓ Certificate: Policy attached"
echo "  ✓ Thing: Name matches client_id"
echo "  ✓ Port: 443 (configured in cloud_config.yaml)"
echo ""
