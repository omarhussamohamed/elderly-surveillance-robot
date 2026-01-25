#!/usr/bin/env python2
"""
AWS IoT Diagnostic Tool
Run this to diagnose connection issues
"""
import socket
import ssl
import paho.mqtt.client as mqtt
import time
import os

print("=" * 60)
print("AWS IoT DIAGNOSTIC TOOL")
print("=" * 60)

# Configuration
ENDPOINT = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
PORT = 8883
CLIENT_ID = "robot"
CA_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
CERT_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
KEY_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"

def check_certificates():
    print("\n1. Checking certificate files...")
    files = [
        (CA_FILE, "Root CA"),
        (CERT_FILE, "Device Certificate"),
        (KEY_FILE, "Private Key")
    ]
    
    all_ok = True
    for path, name in files:
        if os.path.exists(path):
            print("   ✓ {}: {}".format(name, path))
        else:
            print("   ✗ {} NOT FOUND: {}".format(name, path))
            all_ok = False
    
    return all_ok

def test_network():
    print("\n2. Testing network connectivity...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ENDPOINT, PORT))
        sock.close()
        print("   ✓ TCP connection to {}:{} successful".format(ENDPOINT, PORT))
        return True
    except Exception as e:
        print("   ✗ TCP connection failed: {}".format(e))
        return False

def test_tls():
    print("\n3. Testing TLS handshake...")
    try:
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
        context.load_verify_locations(CA_FILE)
        context.load_cert_chain(CERT_FILE, KEY_FILE)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_REQUIRED
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        tls_sock = context.wrap_socket(sock, server_hostname=ENDPOINT)
        tls_sock.connect((ENDPOINT, PORT))
        tls_sock.close()
        print("   ✓ TLS handshake successful")
        return True
    except Exception as e:
        print("   ✗ TLS handshake failed: {}".format(e))
        return False

def test_mqtt():
    print("\n4. Testing MQTT connection...")
    
    def on_connect(client, userdata, flags, rc):
        print("   Connection result: {}".format(rc))
        if rc == 0:
            print("   ✓ MQTT connection successful!")
            print("   Client ID: {}".format(client._client_id))
        else:
            print("   ✗ MQTT connection failed")
            error_codes = {
                1: "Incorrect protocol version",
                2: "Invalid client identifier",
                3: "Server unavailable",
                4: "Bad username or password",
                5: "Not authorized - CHECK AWS IoT POLICY!"
            }
            print("   Error: {}".format(error_codes.get(rc, "Unknown error: {}".format(rc))))
    
    def on_disconnect(client, userdata, rc):
        print("   Disconnected: {}".format(rc))
    
    try:
        client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311)
        
        # Setup TLS
        client.tls_set(
            ca_certs=CA_FILE,
            certfile=CERT_FILE,
            keyfile=KEY_FILE,
            tls_version=ssl.PROTOCOL_TLSv1_2
        )
        client.tls_insecure_set(True)  # Disable hostname verification
        
        # Set callbacks
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        
        print("   Connecting to {}:{} as '{}'...".format(ENDPOINT, PORT, CLIENT_ID))
        client.connect_async(ENDPOINT, PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(5)
        
        client.loop_stop()
        client.disconnect()
        
    except Exception as e:
        print("   ✗ MQTT error: {}".format(e))
        return False
    
    return True

def main():
    print("Endpoint: {}".format(ENDPOINT))
    print("Client ID: {}".format(CLIENT_ID))
    print("Port: {}".format(PORT))
    
    # Run tests
    certs_ok = check_certificates()
    if not certs_ok:
        print("\n✗ Certificate files missing. Cannot proceed.")
        return
    
    network_ok = test_network()
    if not network_ok:
        print("\n✗ Network issues detected.")
        return
    
    tls_ok = test_tls()
    if not tls_ok:
        print("\n✗ TLS issues detected.")
        return
    
    mqtt_ok = test_mqtt()
    
    print("\n" + "=" * 60)
    print("DIAGNOSTIC SUMMARY:")
    print("=" * 60)
    print("Certificates: {}".format("✓ OK" if certs_ok else "✗ FAILED"))
    print("Network:      {}".format("✓ OK" if network_ok else "✗ FAILED"))
    print("TLS:          {}".format("✓ OK" if tls_ok else "✗ FAILED"))
    print("MQTT:         {}".format("✓ OK" if mqtt_ok else "✗ FAILED"))
    
    if not mqtt_ok:
        print("\n" + "=" * 60)
        print("MOST LIKELY ISSUE: AWS IoT Policy")
        print("=" * 60)
        print("Go to AWS IoT Console → Secure → Policies")
        print("1. Create a new policy with these permissions:")
        print("""
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "*"
    },
    {
      "Effect": "Allow", 
      "Action": "iot:Publish",
      "Resource": "*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Receive",
      "Resource": "*"
    }
  ]
}
""")
        print("2. Attach the policy to your certificate")
        print("3. Make sure your certificate is 'Active'")
        print("4. Make sure certificate is attached to Thing 'robot'")
        print("=" * 60)

if __name__ == "__main__":
    main()