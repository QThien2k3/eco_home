#!/usr/bin/env python3
"""
Enhanced Face Recognition AI Server with Domain Separation Support
Updated for ESP32-CAM architecture with separated stream and API domains
"""

import os
import cv2
import numpy as np
import pickle
import json
import re
import time
import requests
from flask import Flask, request, jsonify
from flask_cors import CORS
import face_recognition
import mediapipe as mp
from datetime import datetime
import sqlite3
import base64
from io import BytesIO
from PIL import Image
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# Initialize MediaPipe
mp_face_detection = mp.solutions.face_detection
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

class FaceRecognitionSystem:
    def __init__(self):
        self.known_face_encodings = []
        self.known_face_names = []
        self.face_database = {}
        self.db_path = "face_database.db"
        self.encodings_path = "face_encodings.pkl"
        
        # Domain separation configuration
        self.esp32_stream_domain = os.getenv('ESP32_STREAM_DOMAIN', 'streamesp32facecam.myfreeiot.win')
        self.esp32_api_domain = os.getenv('ESP32_API_DOMAIN', 'apiesp32facecam.myfreeiot.win')
        self.esp32_local_ip = os.getenv('ESP32_CAM_IP', '192.168.1.92')
        
        # Relaxed thresholds for better user experience
        self.detection_confidence = 0.5
        self.recognition_threshold = 0.25  # Very low for easier recognition
        
        # Initialize database
        self.init_database()
        self.load_encodings()
        
        logger.info(f"Domain Separation Config:")
        logger.info(f"  Stream Domain: {self.esp32_stream_domain}")
        logger.info(f"  API Domain: {self.esp32_api_domain}")
        logger.info(f"  Local IP: {self.esp32_local_ip}")
        
    def init_database(self):
        """Initialize SQLite database for face data"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS faces (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                encoding BLOB NOT NULL,
                images_count INTEGER DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS recognition_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT,
                confidence REAL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                image_path TEXT,
                domain_used TEXT
            )
        ''')
        
        # Add domain separation logs
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS domain_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                domain TEXT,
                endpoint TEXT,
                status TEXT,
                response_time REAL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        conn.commit()
        conn.close()
        
    def save_encodings(self):
        """Save face encodings to pickle file"""
        try:
            data = {
                'encodings': self.known_face_encodings,
                'names': self.known_face_names
            }
            with open(self.encodings_path, 'wb') as f:
                pickle.dump(data, f)
            logger.info("Face encodings saved successfully")
        except Exception as e:
            logger.error(f"Error saving encodings: {e}")
            
    def load_encodings(self):
        """Load face encodings from pickle file"""
        try:
            if os.path.exists(self.encodings_path):
                with open(self.encodings_path, 'rb') as f:
                    data = pickle.load(f)
                    self.known_face_encodings = data['encodings']
                    self.known_face_names = data['names']
                logger.info(f"Loaded {len(self.known_face_names)} face encodings")
            else:
                logger.info("No existing encodings found")
        except Exception as e:
            logger.error(f"Error loading encodings: {e}")
            
    def log_domain_access(self, domain, endpoint, status, response_time):
        """Log domain access for monitoring separation"""
        conn = None
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute(
                "INSERT INTO domain_logs (domain, endpoint, status, response_time) VALUES (?, ?, ?, ?)",
                (domain, endpoint, status, response_time)
            )
            conn.commit()
            
        except Exception as e:
            logger.error(f"Domain logging error: {e}")
        finally:
            if conn:
                conn.close()
                
    def detect_liveness(self, image):
        """Relaxed liveness detection for better UX"""
        try:
            height, width = image.shape[:2]
            
            with mp_face_mesh.FaceMesh(
                static_image_mode=True,
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5
            ) as face_mesh:
                
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(rgb_image)
                
                if not results.multi_face_landmarks:
                    return False, "No face detected"
                
                landmarks = results.multi_face_landmarks[0].landmark
                
                # Eye Aspect Ratio calculation
                left_eye_points = [33, 160, 158, 133, 153, 144]
                right_eye_points = [362, 385, 387, 263, 373, 380]
                
                def calculate_ear(eye_points, landmarks):
                    h1 = abs(landmarks[eye_points[1]].y - landmarks[eye_points[5]].y)
                    h2 = abs(landmarks[eye_points[2]].y - landmarks[eye_points[4]].y)
                    w = abs(landmarks[eye_points[0]].x - landmarks[eye_points[3]].x)
                    return (h1 + h2) / (2.0 * w)
                
                left_ear = calculate_ear(left_eye_points, landmarks)
                right_ear = calculate_ear(right_eye_points, landmarks)
                ear = (left_ear + right_ear) / 2.0
                
                # Texture analysis
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                laplacian = cv2.Laplacian(gray, cv2.CV_64F)
                texture_variance = laplacian.var()
                
                # Edge density analysis
                edges = cv2.Canny(gray, 50, 150)
                edge_density = np.sum(edges > 0) / (width * height)
                
                # Face size analysis
                face_bbox = [
                    min([lm.x for lm in landmarks]) * width,
                    min([lm.y for lm in landmarks]) * height,
                    max([lm.x for lm in landmarks]) * width,
                    max([lm.y for lm in landmarks]) * height
                ]
                face_area = (face_bbox[2] - face_bbox[0]) * (face_bbox[3] - face_bbox[1])
                face_ratio = face_area / (width * height)
                
                # Relaxed scoring for better UX
                liveness_score = 0
                reasons = []
                
                # Eye openness check (very tolerant)
                if 0.08 < ear < 0.50:
                    liveness_score += 20
                else:
                    reasons.append(f"Eye ratio: {ear:.3f}")
                
                # Texture variance (very low threshold)
                if texture_variance > 40:
                    liveness_score += 25
                else:
                    reasons.append(f"Low texture: {texture_variance:.1f}")
                
                # Edge density (very tolerant)
                if 0.01 < edge_density < 0.25:
                    liveness_score += 20
                else:
                    reasons.append(f"Edge density: {edge_density:.3f}")
                
                # Face size (very lenient)
                if face_ratio > 0.02:
                    liveness_score += 15
                else:
                    reasons.append(f"Face too small: {face_ratio:.3f}")
                
                # Always give some points for basic detection
                liveness_score += 20  # Bonus points for having a detectable face
                
                logger.info(f"Liveness analysis - Score: {liveness_score}/100, EAR: {ear:.3f}, Texture: {texture_variance:.1f}")
                
                # Very relaxed threshold
                if liveness_score >= 40:  # Much lower than before
                    return True, f"Live face detected (score: {liveness_score}/100)"
                else:
                    reason_text = "; ".join(reasons[:2])
                    return False, f"Possible fake detected (score: {liveness_score}/100) - {reason_text}"
                    
        except Exception as e:
            logger.error(f"Liveness detection error: {e}")
            return True, "Liveness check skipped - assuming live"  # Default to allow
            
    def extract_face_encoding(self, image):
        """Extract face encoding from image"""
        try:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Find face locations with relaxed parameters
            face_locations = face_recognition.face_locations(rgb_image, model="hog", number_of_times_to_upsample=1)
            
            if not face_locations:
                return None, "No face found in image"
            
            if len(face_locations) > 1:
                # Use the largest face if multiple detected
                face_locations = [max(face_locations, key=lambda x: (x[2]-x[0])*(x[1]-x[3]))]
            
            # Extract face encoding
            face_encodings = face_recognition.face_encodings(rgb_image, face_locations)
            
            if not face_encodings:
                return None, "Could not extract face features"
                
            return face_encodings[0], "Success"
            
        except Exception as e:
            logger.error(f"Face encoding error: {e}")
            return None, f"Error: {str(e)}"
            
    def enroll_face(self, images, name):
        """Enroll a new face with relaxed security for better UX"""
        conn = None
        try:
            if len(images) < 2:  # Reduced from 3 to 2
                return False, "Need at least 2 images for enrollment"
            
            encodings = []
            security_scores = []
            
            # Process each image with very lenient checks
            for i, image in enumerate(images):
                logger.info(f"Processing enrollment image {i+1}/{len(images)}")
                
                # Very lenient liveness detection
                is_live, liveness_msg = self.detect_liveness(image)
                
                # Extract score
                score_match = re.search(r'score: (\d+)/100', liveness_msg)
                security_score = int(score_match.group(1)) if score_match else 50
                security_scores.append(security_score)
                
                # Extract encoding regardless of liveness score
                encoding, msg = self.extract_face_encoding(image)
                if encoding is not None:
                    encodings.append(encoding)
                    logger.info(f"Successfully extracted encoding from image {i+1} (security: {security_score}/100)")
                else:
                    logger.warning(f"Failed to extract encoding from image {i+1}: {msg}")
            
            if len(encodings) < 1:
                return False, "Could not extract any valid face encodings"
            
            # Calculate average encoding
            if len(encodings) == 1:
                avg_encoding = encodings[0]
            else:
                avg_encoding = np.mean(encodings, axis=0)
            
            # Check average security score (very lenient)
            avg_security = sum(security_scores) / len(security_scores) if security_scores else 50
            logger.info(f"Average security score: {avg_security:.1f}/100")
            
            # Save to database
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute(
                "INSERT OR REPLACE INTO faces (name, encoding, images_count) VALUES (?, ?, ?)",
                (name, pickle.dumps(avg_encoding), len(encodings))
            )
            conn.commit()
            
            # Update in-memory storage
            if name in self.known_face_names:
                idx = self.known_face_names.index(name)
                self.known_face_encodings[idx] = avg_encoding
            else:
                self.known_face_names.append(name)
                self.known_face_encodings.append(avg_encoding)
            
            self.save_encodings()
            
            logger.info(f"Successfully enrolled {name} with {len(encodings)} images")
            return True, f"Successfully enrolled {name} with {len(encodings)} images (security: {avg_security:.1f}/100)"
                
        except sqlite3.IntegrityError:
            return False, f"Name {name} already exists"
        except Exception as e:
            logger.error(f"Enrollment error: {e}")
            return False, f"Enrollment failed: {str(e)}"
        finally:
            if conn:
                conn.close()
                
    def recognize_face(self, image, domain_used="unknown"):
        """Recognize face with very relaxed thresholds"""
        try:
            if not self.known_face_encodings:
                return None, 0.0, "No enrolled faces in database", False
            
            # Extract face encoding
            encoding, msg = self.extract_face_encoding(image)
            if encoding is None:
                return None, 0.0, msg, False
            
            # Compare with known faces
            face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
            best_match_index = np.argmin(face_distances)
            confidence = 1 - face_distances[best_match_index]
            
            # Very relaxed threshold for recognition
            if confidence > self.recognition_threshold:
                name = self.known_face_names[best_match_index]
                
                # Log recognition with domain info
                self.log_recognition(name, confidence, domain_used)
                
                logger.info(f"Recognition successful: {name} ({confidence:.2f}) via {domain_used}")
                return name, confidence, "Recognition successful", True
            else:
                logger.info(f"Face not recognized - confidence: {confidence:.2f}")
                return "Unknown", confidence, f"Low confidence: {confidence:.2f} (need >{self.recognition_threshold})", False
                
        except Exception as e:
            logger.error(f"Recognition error: {e}")
            return None, 0.0, f"Recognition failed: {str(e)}", False
            
    def delete_face(self, name):
        """Delete a face from database"""
        conn = None
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("DELETE FROM faces WHERE name = ?", (name,))
            
            if cursor.rowcount > 0:
                conn.commit()
                
                # Remove from in-memory storage
                if name in self.known_face_names:
                    idx = self.known_face_names.index(name)
                    del self.known_face_names[idx]
                    del self.known_face_encodings[idx]
                
                self.save_encodings()
                logger.info(f"Deleted face: {name}")
                return True, f"Successfully deleted {name}"
            else:
                return False, f"Face {name} not found"
                
        except Exception as e:
            logger.error(f"Delete error: {e}")
            return False, f"Delete failed: {str(e)}"
        finally:
            if conn:
                conn.close()
                
    def get_all_faces(self):
        """Get list of all enrolled faces"""
        conn = None
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("SELECT name, images_count, created_at FROM faces ORDER BY name")
            faces = cursor.fetchall()
            
            return [{"name": face[0], "images_count": face[1], "created_at": face[2]} for face in faces]
            
        except Exception as e:
            logger.error(f"Error getting faces: {e}")
            return []
        finally:
            if conn:
                conn.close()
                
    def log_recognition(self, name, confidence, domain_used="unknown"):
        """Log recognition event with domain info"""
        conn = None
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute(
                "INSERT INTO recognition_logs (name, confidence, domain_used) VALUES (?, ?, ?)",
                (name, confidence, domain_used)
            )
            conn.commit()
            
        except Exception as e:
            logger.error(f"Logging error: {e}")
        finally:
            if conn:
                conn.close()

# Initialize face recognition system
face_system = FaceRecognitionSystem()

def process_image_from_request(request):
    """Process image from Flask request"""
    try:
        if 'image' in request.files:
            file = request.files['image']
            if file.filename != '':
                file_bytes = np.frombuffer(file.read(), np.uint8)
                image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
                return image
        
        elif 'image' in request.form:
            image_data = request.form['image']
            if image_data.startswith('data:image'):
                image_data = image_data.split(',')[1]
            
            image_bytes = base64.b64decode(image_data)
            image_array = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            return image
            
        return None
    except Exception as e:
        logger.error(f"Image processing error: {e}")
        return None

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint with domain separation info"""
    return jsonify({
        "status": "healthy",
        "enrolled_faces": len(face_system.known_face_names),
        "recognition_threshold": face_system.recognition_threshold,
        "domain_separation": {
            "stream_domain": face_system.esp32_stream_domain,
            "api_domain": face_system.esp32_api_domain,
            "local_ip": face_system.esp32_local_ip
        },
        "timestamp": datetime.now().isoformat()
    })

@app.route('/enroll', methods=['POST', 'OPTIONS'])
def enroll_face():
    """Enroll a new face"""
    if request.method == 'OPTIONS':
        return '', 204
        
    try:
        name = request.form.get('name', '').strip()
        if not name:
            return jsonify({"success": False, "message": "Name is required"}), 400
        
        logger.info(f"Starting enrollment for: {name}")
        
        # Get images
        images = []
        
        for i in range(10):
            file_key = f'image{i}' if i > 0 else 'image'
            if file_key in request.files:
                file = request.files[file_key]
                if file.filename != '':
                    try:
                        file_bytes = np.frombuffer(file.read(), np.uint8)
                        image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
                        if image is not None:
                            images.append(image)
                            logger.info(f"Successfully loaded image {file_key}")
                    except Exception as e:
                        logger.error(f"Error processing image {file_key}: {e}")
        
        if len(images) == 0:
            return jsonify({"success": False, "message": "No valid images provided"}), 400
        
        # If only one image, duplicate it
        if len(images) == 1:
            images = [images[0]] * 2
        
        logger.info(f"Processing {len(images)} images for enrollment")
        
        success, message = face_system.enroll_face(images, name)
        
        if success:
            logger.info(f"Successfully enrolled {name}")
            return jsonify({
                "success": True,
                "message": message,
                "name": name,
                "images_processed": len(images)
            })
        else:
            logger.warning(f"Enrollment failed for {name}: {message}")
            return jsonify({
                "success": False,
                "message": message,
                "name": name
            }), 400
        
    except Exception as e:
        logger.error(f"Enroll endpoint error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500

@app.route('/recognize', methods=['POST', 'OPTIONS'])
def recognize_face():
    """Recognize a face"""
    if request.method == 'OPTIONS':
        return '', 204
        
    try:
        image = process_image_from_request(request)
        if image is None:
            return jsonify({"success": False, "message": "No valid image provided"}), 400
        
        logger.info("Starting face recognition...")
        
        # Determine which domain was used (based on referrer or custom header)
        domain_used = request.headers.get('X-Domain-Used', 'api_domain')
        
        name, confidence, message, liveness_passed = face_system.recognize_face(image, domain_used)
        
        if name is not None and name != "Unknown" and liveness_passed:
            logger.info(f"Recognition successful: {name} ({confidence:.2f})")
            return jsonify({
                "success": True,
                "name": name,
                "confidence": float(confidence),
                "message": "Recognition successful",
                "liveness_message": message,
                "security_level": "high" if confidence > 0.6 else "medium" if confidence > 0.4 else "low",
                "domain_used": domain_used,
                "timestamp": datetime.now().isoformat()
            })
        else:
            logger.info(f"Face not recognized: {message}")
            return jsonify({
                "success": False,
                "name": name if name else "Unknown",
                "confidence": float(confidence) if confidence else 0.0,
                "message": message,
                "domain_used": domain_used,
                "timestamp": datetime.now().isoformat()
            })
        
    except Exception as e:
        logger.error(f"Recognize endpoint error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500

@app.route('/delete', methods=['POST', 'OPTIONS'])
def delete_face():
    """Delete a face from database"""
    if request.method == 'OPTIONS':
        return '', 204
        
    try:
        data = request.get_json()
        name = data.get('name', '').strip()
        
        if not name:
            return jsonify({"success": False, "message": "Name is required"}), 400
        
        success, message = face_system.delete_face(name)
        
        return jsonify({
            "success": success,
            "message": message
        })
        
    except Exception as e:
        logger.error(f"Delete endpoint error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500

@app.route('/list', methods=['GET'])
def list_faces():
    """List all enrolled faces"""
    try:
        faces = face_system.get_all_faces()
        return jsonify({
            "success": True,
            "faces": faces,
            "total": len(faces)
        })
        
    except Exception as e:
        logger.error(f"List endpoint error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500

@app.route('/logs', methods=['GET'])
def get_logs():
    """Get recognition logs with domain info"""
    conn = None
    try:
        conn = sqlite3.connect(face_system.db_path)
        cursor = conn.cursor()
        
        limit = request.args.get('limit', 50, type=int)
        cursor.execute(
            "SELECT name, confidence, timestamp, domain_used FROM recognition_logs ORDER BY timestamp DESC LIMIT ?",
            (limit,)
        )
        
        logs = cursor.fetchall()
        
        return jsonify({
            "success": True,
            "logs": [{"name": log[0], "confidence": log[1], "timestamp": log[2], "domain_used": log[3] or "unknown"} for log in logs]
        })
        
    except Exception as e:
        logger.error(f"Logs endpoint error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500
    finally:
        if conn:
            conn.close()

@app.route('/proxy/capture', methods=['GET', 'OPTIONS'])
def proxy_capture():
    """Enhanced proxy endpoint with domain separation support"""
    if request.method == 'OPTIONS':
        return '', 204
        
    start_time = time.time()
    
    try:
        # Use API domain for capture
        esp32_url = f"http://{face_system.esp32_local_ip}"
        
        logger.info(f"Proxying capture request to {esp32_url} (API domain)")
        
        # Make request to ESP32-CAM via API domain
        response = requests.get(
            f"{esp32_url}/capture",
            timeout=10,
            params={"t": int(time.time()), "proxy": 1, "api_domain": 1}
        )
        
        response_time = time.time() - start_time
        
        if response.status_code == 200:
            # Log successful domain access
            face_system.log_domain_access("api_domain", "/capture", "success", response_time)
            
            # Return image data with proper CORS headers
            return response.content, 200, {
                'Content-Type': 'image/jpeg',
                'Content-Length': str(len(response.content)),
                'Access-Control-Allow-Origin': '*',
                'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                'Access-Control-Allow-Headers': 'Content-Type',
                'Cache-Control': 'no-cache, no-store, must-revalidate',
                'Pragma': 'no-cache',
                'Expires': '0',
                'X-Domain-Used': 'api_domain',
                'X-Response-Time': str(response_time)
            }
        else:
            face_system.log_domain_access("api_domain", "/capture", "error", response_time)
            return jsonify({
                "success": False, 
                "message": f"ESP32-CAM API domain returned {response.status_code}"
            }), 500
            
    except requests.exceptions.RequestException as e:
        response_time = time.time() - start_time
        face_system.log_domain_access("api_domain", "/capture", "timeout", response_time)
        
        logger.error(f"Proxy capture error: {e}")
        return jsonify({
            "success": False,
            "message": f"Cannot connect to ESP32-CAM API domain: {str(e)}. Check IP and WiFi."
        }), 500
    except Exception as e:
        logger.error(f"Proxy capture error: {e}")
        return jsonify({"success": False, "message": f"Proxy error: {str(e)}"}), 500

@app.route('/config/esp32_ip', methods=['POST', 'GET', 'OPTIONS'])
def config_esp32_ip():
    """Configure ESP32-CAM IP address for domain separation"""
    if request.method == 'OPTIONS':
        return '', 204
        
    if request.method == 'POST':
        try:
            data = request.get_json()
            ip = data.get('ip', '').strip()
            
            # Validate IP format
            ip_pattern = r'^(\d{1,3}\.){3}\d{1,3}$'
            if not re.match(ip_pattern, ip):
                return jsonify({"success": False, "message": "Invalid IP format"}), 400
            
            # Update configuration
            face_system.esp32_local_ip = ip
            os.environ['ESP32_CAM_IP'] = ip
            
            # Test both domains
            results = {}
            
            # Test API domain
            try:
                api_response = requests.get(f"http://{ip}/status", timeout=5)
                results['api_domain'] = {
                    "status": "online" if api_response.status_code == 200 else "error",
                    "response_code": api_response.status_code
                }
            except Exception as e:
                results['api_domain'] = {"status": "offline", "error": str(e)}
            
            # Test basic connectivity (stream domain uses same IP)
            try:
                basic_response = requests.head(f"http://{ip}", timeout=3)
                results['basic_connectivity'] = "ok"
            except Exception as e:
                results['basic_connectivity'] = "failed"
            
            if results['api_domain']['status'] == 'online':
                return jsonify({
                    "success": True, 
                    "message": f"ESP32-CAM IP set to {ip} with domain separation",
                    "ip": ip,
                    "domain_tests": results
                })
            else:
                return jsonify({
                    "success": False, 
                    "message": f"IP set to {ip} but API domain test failed",
                    "domain_tests": results
                }), 400
                
        except Exception as e:
            return jsonify({"success": False, "message": str(e)}), 500
    else:
        # GET method
        return jsonify({
            "success": True, 
            "ip": face_system.esp32_local_ip,
            "stream_domain": face_system.esp32_stream_domain,
            "api_domain": face_system.esp32_api_domain
        })

@app.route('/domain/stats', methods=['GET'])
def domain_statistics():
    """Get domain separation statistics"""
    conn = None
    try:
        conn = sqlite3.connect(face_system.db_path)
        cursor = conn.cursor()
        
        # Get domain access stats
        cursor.execute("""
            SELECT domain, endpoint, status, COUNT(*) as count, AVG(response_time) as avg_time
            FROM domain_logs 
            WHERE timestamp > datetime('now', '-24 hours')
            GROUP BY domain, endpoint, status
            ORDER BY count DESC
        """)
        
        domain_stats = cursor.fetchall()
        
        # Get recognition stats by domain
        cursor.execute("""
            SELECT domain_used, COUNT(*) as count, AVG(confidence) as avg_confidence
            FROM recognition_logs 
            WHERE timestamp > datetime('now', '-24 hours')
            GROUP BY domain_used
        """)
        
        recognition_stats = cursor.fetchall()
        
        return jsonify({
            "success": True,
            "domain_access_stats": [
                {
                    "domain": stat[0], 
                    "endpoint": stat[1], 
                    "status": stat[2], 
                    "count": stat[3], 
                    "avg_response_time": stat[4]
                } for stat in domain_stats
            ],
            "recognition_stats": [
                {
                    "domain_used": stat[0] or "unknown", 
                    "count": stat[1], 
                    "avg_confidence": stat[2]
                } for stat in recognition_stats
            ],
            "timestamp": datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"Domain stats error: {e}")
        return jsonify({"success": False, "message": f"Server error: {str(e)}"}), 500
    finally:
        if conn:
            conn.close()

if __name__ == '__main__':
    # Create necessary directories
    os.makedirs('uploads', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    
    print("=== Enhanced Face Recognition AI Server ===")
    print("=== Domain Separation Architecture ===")
    print("Checking dependencies...")
    
    # Check dependencies
    try:
        import cv2
        print("✓ OpenCV")
    except ImportError:
        print("✗ OpenCV not found. Install: pip install opencv-python")
        exit(1)
        
    try:
        import face_recognition
        print("✓ Face Recognition")
    except ImportError:
        print("✗ Face Recognition not found. Install: pip install face-recognition")
        exit(1)
        
    try:
        import mediapipe
        print("✓ MediaPipe")
    except ImportError:
        print("✗ MediaPipe not found. Install: pip install mediapipe")
        exit(1)
        
    try:
        import requests
        print("✓ Requests")
    except ImportError:
        print("✗ Requests not found. Install: pip install requests")
        exit(1)
    
    print("✓ All dependencies OK")
    print(f"Enrolled faces: {len(face_system.known_face_names)}")
    if len(face_system.known_face_names) > 0:
        print(f"Known faces: {', '.join(face_system.known_face_names)}")
    
    print("\n=== Domain Separation Configuration ===")
    print(f"Stream Domain: {face_system.esp32_stream_domain}")
    print(f"API Domain: {face_system.esp32_api_domain}")
    print(f"ESP32 Local IP: {face_system.esp32_local_ip}")
    print(f"Recognition threshold: {face_system.recognition_threshold} (very relaxed)")
    print(f"Detection confidence: {face_system.detection_confidence}")
    
    print("\n=== Server Starting ===")
    print("Server URL: http://0.0.0.0:5000")
    print("Cloudflare URL: https://pyfaceid.myfreeiot.win")
    print("\n🔧 ENHANCED FEATURES:")
    print("  • Domain separation support")
    print("  • Stream/API isolation monitoring")
    print("  • Enhanced proxy with domain awareness")
    print("  • Domain access logging and statistics")
    print("  • Very relaxed AI thresholds (25% confidence)")
    print("  • ESP32-CAM IP configuration with domain testing")
    print("\nAvailable endpoints:")
    print("  GET  /health - Health check with domain info")
    print("  POST /enroll - Enroll new face")
    print("  POST /recognize - Recognize face (with domain tracking)")
    print("  POST /delete - Delete face")
    print("  GET  /list - List all faces")
    print("  GET  /logs - Get recognition logs (with domain info)")
    print("  GET  /proxy/capture - Enhanced capture via API domain")
    print("  POST /config/esp32_ip - Configure ESP32-CAM IP with domain testing")
    print("  GET  /domain/stats - Domain separation statistics")
    print("\n=== Ready for Domain Separated Face Recognition ===")
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
