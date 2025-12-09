// =============================================================================
// Standard C/C++ Library Includes
// =============================================================================
#include <math.h>      // Mathematical functions (sin, cos, sqrt, atan2, etc.)
#include <stdio.h>     // Standard I/O functions (sprintf for text formatting)
#include <stdlib.h>    // Standard library functions (exit, etc.)
#include <string.h>    // String manipulation functions
#include <vector>      // STL vector for dynamic arrays
#include <algorithm>
#include <map>
#include <string>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// OpenGL/GLUT Includes
// =============================================================================
#ifdef _WIN32
#include <windows.h>
#include <GL/freeglut.h>   // Windows FreeGLUT header
#else
#include <GL/glut.h>       // Linux/Mac GLUT header
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NOEXCEPTION
#include "third_party/tiny_gltf.h"

/*
 =============================================================================
 Chrono Heist - 3D Time-Traveling Adventure Game
 =============================================================================
 
 DESCRIPTION:
 ------------
 Chrono Heist is a 3D adventure where a time-traveling agent retrieves temporal
 artifacts across two scenes: a Neo-Tokyo research facility and an ancient
 Egyptian temple. The player avoids obstacles, collects artifacts, and reaches
 the exit portal. Both first-person and third-person cameras are supported.
 
 KEY FEATURES:
 ------------
 - Two distinct environments: Neo-Tokyo Research Facility and Ancient Egyptian Temple
 - Dual camera modes: First-person and Third-person (toggle with C)
 - Exactly 2 light sources: Key Light (oscillating intensity) and Sentinel Light (moving)
 - Collectibles: Temporal Crystals (Neo-Tokyo) and Golden Scarabs (Temple)
 - Obstacles: Laser Grids, Motion Detectors, Security Cameras, Pressure Plates, etc.
 - Interactive objects: Control Console, Exit Portals
 - HUD: Score, health, collected items, objective hints
 - Animations and SFX triggers on all interactions
 
 TECHNICAL DETAILS:
 -----------------
 - Uses GLUT for window management and input handling
 - OpenGL for 3D rendering
 - Collision detection: AABB and sphere-distance checks
 - Animation system using GLUT timer callbacks
 - Emissive materials for collectibles and portals (no extra lights)
 
 AUTHOR:
 -------
 DMET 502 Computer Graphics, Winter 2025
 German University in Cairo - Media Engineering and Technology
 
 =============================================================================
*/

// =============================================================================
// Constants and Macros
// =============================================================================
#define GLUT_KEY_ESCAPE 27
#define DEG2RAD(a) (a * 0.0174532925)
#define RAD2DEG(a) (a * 57.2957795)

// Window dimensions
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;

// World boundaries
const float WORLD_MIN = -30.0f;
const float WORLD_MAX = 30.0f;
const float GROUND_Y = 0.0f;
const float CEILING_Y = 20.0f;

// Player properties
const float PLAYER_RADIUS = 0.5f;
const float PLAYER_HEIGHT = 1.8f;
const float PLAYER_SPEED = 0.4f;
const float PLAYER_JUMP_FORCE = 0.3f;
const float GRAVITY = -0.02f;

// Game constants
const int MAX_HEALTH = 100;
const int CRYSTAL_POINTS = 500;
const int SCARAB_POINTS = 500;
const int SCARABS_REQUIRED = 4;
const int MAX_CRYSTALS = 6;
const int MAX_SCARABS = 6;

// Camera distances
const float FIRST_PERSON_DISTANCE = 0.0f;
const float THIRD_PERSON_DISTANCE = 8.0f;   // Distance behind player
const float THIRD_PERSON_HEIGHT = 3.0f;     // Height above player

const char* TEXTURE_DIR = "textures/";
const char* MODERN_TOKYO_BASE = "textures/modern_tokyo/basecolor.png";
const char* ANCIENT_EGYPT_BASE = "textures/ancient_egypt/basecolor.jpg";
const char* CHARACTER_MODEL_PATH = "main_character/squall.obj";
const char* CHARACTER_TEXTURE_PATH = "main_character/ps2021_C01.png";

struct TextureResource {
    GLuint id;
    int width;
    int height;
    std::string source;

    TextureResource() {
        id = 0;
        width = 0;
        height = 0;
    }

    bool valid() const { return id != 0; }
};

// =============================================================================
// Vector3f Class - 3D Vector Mathematics
// =============================================================================
class Vector3f {
public:
    float x, y, z;

    Vector3f(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f) {
        x = _x;
        y = _y;
        z = _z;
    }

    Vector3f operator+(const Vector3f& v) const {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }

    Vector3f operator-(const Vector3f& v) const {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }

    Vector3f operator*(float n) const {
        return Vector3f(x * n, y * n, z * n);
    }

    Vector3f operator/(float n) const {
        return Vector3f(x / n, y / n, z / n);
    }

    float length() const {
        return sqrt(x * x + y * y + z * z);
    }

    float lengthSquared() const {
        return x * x + y * y + z * z;
    }

    Vector3f unit() const {
        float len = length();
        if (len < 0.0001f) return Vector3f(0, 0, 0);
        return *this / len;
    }

    Vector3f cross(const Vector3f& v) const {
        return Vector3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    float dot(const Vector3f& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
};

struct CharacterMeshData {
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<unsigned int> indices;
    TextureResource albedo;
    Vector3f boundsMin;
    Vector3f boundsMax;
    Vector3f boundsCenter;
    float scale;
    bool loaded;

    CharacterMeshData() {
        boundsMin = Vector3f();
        boundsMax = Vector3f();
        boundsCenter = Vector3f();
        scale = 1.0f;
        loaded = false;
    }
};

// Generic OBJ Mesh structure for props (scarab, camera, console)
struct OBJMesh {
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<unsigned int> indices;
    TextureResource albedo;
    Vector3f boundsMin;
    Vector3f boundsMax;
    Vector3f boundsCenter;
    float scale;
    bool loaded;

    OBJMesh() {
        boundsMin = Vector3f();
        boundsMax = Vector3f();
        boundsCenter = Vector3f();
        scale = 1.0f;
        loaded = false;
    }
    
    void computeBounds() {
        if (positions.empty()) return;
        boundsMin = Vector3f(positions[0], positions[1], positions[2]);
        boundsMax = boundsMin;
        for (size_t i = 0; i < positions.size(); i += 3) {
            float x = positions[i], y = positions[i + 1], z = positions[i + 2];
            boundsMin.x = std::min(boundsMin.x, x);
            boundsMin.y = std::min(boundsMin.y, y);
            boundsMin.z = std::min(boundsMin.z, z);
            boundsMax.x = std::max(boundsMax.x, x);
            boundsMax.y = std::max(boundsMax.y, y);
            boundsMax.z = std::max(boundsMax.z, z);
        }
        boundsCenter = Vector3f(
            (boundsMin.x + boundsMax.x) * 0.5f,
            (boundsMin.y + boundsMax.y) * 0.5f,
            (boundsMin.z + boundsMax.z) * 0.5f
        );
    }
};

// Model paths for props
const char* SCARAB_MODEL_PATH = "scarabnew/Scarab Beetle/scarab.obj";
const char* SCARAB_TEXTURE_PATH = "scarabnew/Scarab Beetle/scarab_diff.png";
const char* CAMERA_MODEL_PATH = "Security Camera/security_camera.obj";
const char* CAMERA_TEXTURE_PATH = "";  // No texture for now
const char* CONSOLE_MODEL_PATH = "sci-fi-console/source/OBJECT 9/OBJECT 9.obj";
const char* CONSOLE_TEXTURE_PATH = "sci-fi-console/source/OBJECT 9/Texture 1.jpg";
const char* GUARD_MODEL_PATH = "robot_guard_texture/GuardReploid/GuardReploid.obj";
const char* GUARD_TEXTURE_PATH = "robot_guard_texture/GuardReploid/GuardReploid.png";
const char* ANUBIS_MODEL_PATH = "ancient_egypt_enemy_texture/FFXIII-LR_X360_MONSTER_Anubys/FFXIII-LR_X360_MONSTER_Anubys.obj";
const char* ANUBIS_TEXTURE_PATH = "ancient_egypt_enemy_texture/FFXIII-LR_X360_MONSTER_Anubys/FFXIII-LR_X360_MONSTER_Anubys_Body_D.png";

// New model paths for Neo-Tokyo neon sign and Egyptian tomb
const char* NEON_SIGN_MODEL_PATH = "japanese-neon-street-sign/source/SIGN.obj";
const char* NEON_SIGN_TEXTURE_PATH = "japanese-neon-street-sign/textures/sign_Albedo.tga.png";
const char* NEON_SIGN_EMISSIVE_PATH = "japanese-neon-street-sign/textures/emiss.jpg";
const char* TOMB_MODEL_PATH = "coffin-of-akhenaten-tomb-kv55/source/akhenatencoffin.obj";
const char* TOMB_TEXTURE_PATH = "coffin-of-akhenaten-tomb-kv55/source/akhenatencoffin.1001.jpg";
const char* HIEROGLYPHIC_TEXTURE_PATH = "egyptian-hieroglyphics-background-with-flat-design/388725-PCDZ9P-153.jpg";
const char* COLUMN_MODEL_PATH = "column/Nothern Undead Asylum Pillar (Alternate)/o8601.obj";
const char* COLUMN_TEXTURE_PATH = "column/Nothern Undead Asylum Pillar (Alternate)/m19_B_wall_07.png";

// =============================================================================
// Game State Enumeration
// =============================================================================
enum GameState {
    STATE_MENU,
    STATE_NEO_TOKYO,
    STATE_TEMPLE,
    STATE_WIN,
    STATE_LOSE
};

enum CameraMode {
    CAMERA_FIRST_PERSON,
    CAMERA_THIRD_PERSON
};

// =============================================================================
// Player Class
// =============================================================================
// The player has:
// - position: world position
// - yaw: the direction the player MODEL is facing (updated when moving)
// - pitch: vertical look angle (used in first-person mode)
// - facingYaw: smoothly interpolated facing direction for model rotation
// =============================================================================
class Player {
public:
    Vector3f position;
    Vector3f velocity;
    float yaw;          // Direction player model is facing (degrees, 0 = +Z)
    float pitch;        // Vertical rotation (for first-person look)
    float targetYaw;    // Target yaw for smooth rotation
    bool onGround;
    int health;
    int score;
    int crystalsCollected;
    int scarabsCollected;

    Player() {
        position = Vector3f(0, 2, 0);
        velocity = Vector3f(0, 0, 0);
        yaw = 0.0f;       // Facing forward (positive Z direction)
        targetYaw = 0.0f;
        pitch = 0.0f;     // Look straight ahead initially
        onGround = true;
        health = MAX_HEALTH;
        score = 0;
        crystalsCollected = 0;
        scarabsCollected = 0;
    }

    void update() {
        // Apply gravity
        if (!onGround) {
            velocity.y += GRAVITY;
        }

        // Update position
        position = position + velocity;

        // Ground collision - BUT allow falling into pits in Temple scene
        // This basic check allows falling in pit areas; game logic handles the actual death
        bool overPit = false;
        
        // Moving platform pit area (always present in Temple)
        float mpHalfWidth = 3.5f;   // Slightly larger than actual pit
        float mpHalfLength = 4.5f;
        Vector3f mpBase(-10.0f, GROUND_Y, 10.0f);
        if (position.x > mpBase.x - mpHalfWidth && position.x < mpBase.x + mpHalfWidth &&
            position.z > mpBase.z - mpHalfLength && position.z < mpBase.z + mpHalfLength) {
            overPit = true;
        }
        
        // Pressure plate trap pit area (when door is open)
        float ppHalfSize = 1.5f;  // Pressure plate size
        Vector3f ppBase(5.0f, GROUND_Y, 0.0f);
        if (position.x > ppBase.x - ppHalfSize && position.x < ppBase.x + ppHalfSize &&
            position.z > ppBase.z - ppHalfSize && position.z < ppBase.z + ppHalfSize) {
            overPit = true;  // Allow falling through pressure plate trap
        }
        
        if (!overPit && position.y < GROUND_Y + PLAYER_HEIGHT / 2) {
            position.y = GROUND_Y + PLAYER_HEIGHT / 2;
            velocity.y = 0;
            onGround = true;
        }

        // Ceiling collision
        if (position.y > CEILING_Y - PLAYER_HEIGHT / 2) {
            position.y = CEILING_Y - PLAYER_HEIGHT / 2;
            velocity.y = 0;
        }

        // Boundary collision
        if (position.x < WORLD_MIN) position.x = WORLD_MIN;
        if (position.x > WORLD_MAX) position.x = WORLD_MAX;
        if (position.z < WORLD_MIN) position.z = WORLD_MIN;
        if (position.z > WORLD_MAX) position.z = WORLD_MAX;

        // Damping
        velocity.x *= 0.9f;
        velocity.z *= 0.9f;
        
        // Smoothly interpolate yaw towards targetYaw for smooth model rotation
        float yawDiff = targetYaw - yaw;
        // Handle wrap-around (e.g., going from 350 to 10 degrees)
        while (yawDiff > 180.0f) yawDiff -= 360.0f;
        while (yawDiff < -180.0f) yawDiff += 360.0f;
        yaw += yawDiff * 0.15f;  // Smooth rotation (adjust 0.15 for speed)
        
        // Keep yaw in 0-360 range
        while (yaw > 360.0f) yaw -= 360.0f;
        while (yaw < 0.0f) yaw += 360.0f;
    }

    void jump() {
        if (onGround) {
            velocity.y = PLAYER_JUMP_FORCE;
            onGround = false;
        }
    }

    // Move the player based on input direction and camera orientation
    // direction.z = forward/backward input (-1 to 1)
    // direction.x = left/right input (-1 to 1)
    // cameraForward and cameraRight are the camera's facing directions
    void moveRelativeToCamera(Vector3f direction, Vector3f cameraForward, Vector3f cameraRight) {
        // Normalize direction for consistent speed
        if (direction.lengthSquared() > 0.0001f) {
            direction = direction.unit();
        }
        
        // Calculate world-space movement direction based on camera orientation
        // W/S (direction.z) moves in camera's forward/backward direction
        // A/D (direction.x) moves in camera's left/right direction
        Vector3f moveDir = (cameraForward * direction.z + cameraRight * direction.x) * PLAYER_SPEED;
        
        // Apply velocity
        velocity.x = moveDir.x;
        velocity.z = moveDir.z;
        
        // Update player's facing direction to match movement direction
        // Only update if actually moving
        if (moveDir.lengthSquared() > 0.0001f) {
            // Calculate the angle from the movement direction
            // atan2 gives angle in radians, convert to degrees
            targetYaw = RAD2DEG(atan2(moveDir.x, moveDir.z));
            
            // Keep targetYaw in 0-360 range
            while (targetYaw < 0.0f) targetYaw += 360.0f;
            while (targetYaw > 360.0f) targetYaw -= 360.0f;
        }
    }
};

// Simple AABB used for blocking collisions
struct AABB {
    Vector3f min;
    Vector3f max;
};

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Resolve collision between player (cylindrical approximation) and an AABB in XZ plane with height
bool resolvePlayerAABBCollision(Player& player, const AABB& box) {
    float playerMinY = player.position.y - PLAYER_HEIGHT * 0.5f;
    float playerMaxY = player.position.y + PLAYER_HEIGHT * 0.5f;
    
    // Vertical overlap check
    if (playerMaxY < box.min.y || playerMinY > box.max.y) {
        return false;
    }
    
    float nearestX = clampf(player.position.x, box.min.x, box.max.x);
    float nearestZ = clampf(player.position.z, box.min.z, box.max.z);
    
    float dx = player.position.x - nearestX;
    float dz = player.position.z - nearestZ;
    float distSq = dx * dx + dz * dz;
    
    if (distSq >= PLAYER_RADIUS * PLAYER_RADIUS) {
        return false;
    }
    
    float overlapX = PLAYER_RADIUS - fabsf(dx);
    float overlapZ = PLAYER_RADIUS - fabsf(dz);
    
    if (overlapX < overlapZ) {
        player.position.x += (dx >= 0 ? overlapX : -overlapX);
    } else {
        player.position.z += (dz >= 0 ? overlapZ : -overlapZ);
    }
    return true;
}

// =============================================================================
// Camera Class - Third-Person Camera System
// =============================================================================
// The camera orbits around the player based on mouse input (cameraYaw, cameraPitch).
// Movement is relative to the camera's facing direction, not global axes.
// The player model rotates to face the direction of movement.
// =============================================================================
class Camera {
public:
    CameraMode mode;
    Vector3f eye;           // Camera position in world space
    Vector3f center;        // Point the camera looks at (player position)
    Vector3f up;            // Up vector (always 0,1,0)
    
    // Camera orbit angles (controlled by mouse)
    float cameraYaw;        // Horizontal rotation around player (0-360 degrees)
    float cameraPitch;      // Vertical angle (-60 to 60 degrees)
    
    // Camera distance settings
    float distance;         // Distance from player
    float height;           // Height offset above player
    
    // Smoothing factor for camera movement (0-1, higher = smoother)
    float smoothFactor;

    Camera() {
        mode = CAMERA_THIRD_PERSON;
        eye = Vector3f(0, 8, 10);
        center = Vector3f(0, 2, 0);
        up = Vector3f(0, 1, 0);
        
        // Initialize camera orbit angles
        cameraYaw = 0.0f;       // Start looking at player from behind (0 = behind player)
        cameraPitch = 20.0f;    // Slight downward angle for better view
        
        // Camera positioning
        distance = THIRD_PERSON_DISTANCE;
        height = THIRD_PERSON_HEIGHT;
        
        smoothFactor = 0.1f;
    }

    // Get the forward direction vector based on camera yaw (for movement)
    // This is the direction the camera is facing in the XZ plane
    Vector3f getForwardDirection() {
        float radYaw = DEG2RAD(cameraYaw);
        // Forward is the direction FROM camera TO player (negative of camera offset direction)
        return Vector3f(-sin(radYaw), 0, -cos(radYaw));
    }
    
    // Get the right direction vector based on camera yaw (for strafing)
    // Perpendicular to forward in the XZ plane
    Vector3f getRightDirection() {
        float radYaw = DEG2RAD(cameraYaw);
        // Right is 90 degrees rotated from forward (fixed: was inverted)
        return Vector3f(cos(radYaw), 0, -sin(radYaw));
    }

    void update(Player& player) {
        if (mode == CAMERA_FIRST_PERSON) {
            // =========================================
            // FIRST-PERSON CAMERA
            // =========================================
            // Camera is at player's eye level, looking in player's facing direction
            eye = player.position + Vector3f(0, PLAYER_HEIGHT * 0.9f, 0);
            
            float radYaw = DEG2RAD(player.yaw);
            float radPitch = DEG2RAD(player.pitch);
            
            // Look direction based on player yaw and pitch
            float forwardDist = 5.0f;
            center = eye + Vector3f(
                sin(radYaw) * cos(radPitch) * forwardDist,
                sin(radPitch) * forwardDist,
                cos(radYaw) * cos(radPitch) * forwardDist
            );
        } else {
            // =========================================
            // THIRD-PERSON CAMERA (Main Implementation)
            // =========================================
            // Camera orbits around player based on cameraYaw and cameraPitch
            // The camera stays a fixed distance behind and above the player
            
            // Convert angles to radians
            float radYaw = DEG2RAD(cameraYaw);
            float radPitch = DEG2RAD(cameraPitch);
            
            // Calculate camera position relative to player
            // Camera is positioned on a sphere around the player
            // cameraYaw rotates around Y axis, cameraPitch tilts up/down
            
            // Horizontal distance from player (affected by pitch)
            float horizontalDist = distance * cos(radPitch);
            
            // Vertical offset (height above player + pitch adjustment)
            float verticalOffset = height + distance * sin(radPitch);
            
            // Calculate camera offset from player position
            // Camera is BEHIND player in the direction of cameraYaw
            Vector3f cameraOffset = Vector3f(
                sin(radYaw) * horizontalDist,    // X offset
                verticalOffset,                   // Y offset (height)
                cos(radYaw) * horizontalDist     // Z offset
            );
            
            // Set camera eye position (with optional smoothing)
            Vector3f targetEye = player.position + cameraOffset;
            
            // =========================================
            // CAMERA BOUNDARY CONSTRAINT (Collision + Zoom)
            // =========================================
            // Check if camera would go outside world boundaries
            // If so, smoothly zoom in to keep camera within bounds
            const float BOUNDARY_MARGIN = 1.5f;  // Keep camera this far from walls
            const float MIN_DISTANCE = 2.0f;     // Minimum camera distance from player
            
            float effectiveDistance = distance;
            bool needsZoom = false;
            
            // Check each boundary and calculate required zoom
            if (targetEye.x < WORLD_MIN + BOUNDARY_MARGIN) {
                float overlapX = (WORLD_MIN + BOUNDARY_MARGIN) - targetEye.x;
                float zoomFactorX = 1.0f - (overlapX / horizontalDist);
                effectiveDistance = std::max(MIN_DISTANCE, effectiveDistance * zoomFactorX);
                needsZoom = true;
            }
            if (targetEye.x > WORLD_MAX - BOUNDARY_MARGIN) {
                float overlapX = targetEye.x - (WORLD_MAX - BOUNDARY_MARGIN);
                float zoomFactorX = 1.0f - (overlapX / horizontalDist);
                effectiveDistance = std::max(MIN_DISTANCE, effectiveDistance * zoomFactorX);
                needsZoom = true;
            }
            if (targetEye.z < WORLD_MIN + BOUNDARY_MARGIN) {
                float overlapZ = (WORLD_MIN + BOUNDARY_MARGIN) - targetEye.z;
                float zoomFactorZ = 1.0f - (overlapZ / horizontalDist);
                effectiveDistance = std::max(MIN_DISTANCE, effectiveDistance * zoomFactorZ);
                needsZoom = true;
            }
            if (targetEye.z > WORLD_MAX - BOUNDARY_MARGIN) {
                float overlapZ = targetEye.z - (WORLD_MAX - BOUNDARY_MARGIN);
                float zoomFactorZ = 1.0f - (overlapZ / horizontalDist);
                effectiveDistance = std::max(MIN_DISTANCE, effectiveDistance * zoomFactorZ);
                needsZoom = true;
            }
            
            // If zooming is needed, recalculate camera position with reduced distance
            if (needsZoom) {
                float newHorizontalDist = effectiveDistance * cos(radPitch);
                float newVerticalOffset = height + effectiveDistance * sin(radPitch);
                cameraOffset = Vector3f(
                    sin(radYaw) * newHorizontalDist,
                    newVerticalOffset,
                    cos(radYaw) * newHorizontalDist
                );
                targetEye = player.position + cameraOffset;
            }
            
            // Final hard clamp to ensure camera never goes outside bounds
            targetEye.x = clampf(targetEye.x, WORLD_MIN + 0.5f, WORLD_MAX - 0.5f);
            targetEye.z = clampf(targetEye.z, WORLD_MIN + 0.5f, WORLD_MAX - 0.5f);
            targetEye.y = clampf(targetEye.y, GROUND_Y + 1.0f, CEILING_Y - 1.0f);
            
            eye = targetEye;
            
            // Camera looks at the player (slightly above ground for better framing)
            center = player.position + Vector3f(0, PLAYER_HEIGHT * 0.5f, 0);
        }
    }

    // Apply the camera view using gluLookAt
    void look() {
        gluLookAt(
            eye.x, eye.y, eye.z,        // Camera position
            center.x, center.y, center.z, // Look-at point (player)
            up.x, up.y, up.z            // Up vector
        );
    }

    void toggleMode() {
        mode = (mode == CAMERA_FIRST_PERSON) ? CAMERA_THIRD_PERSON : CAMERA_FIRST_PERSON;
    }
};

// =============================================================================
// Collectible Class
// =============================================================================
class Collectible {
public:
    Vector3f position;
    bool collected;
    float rotation;
    float bobOffset;
    int type;  // 0 = Crystal, 1 = Scarab

    Collectible(Vector3f pos, int t) {
        position = pos;
        collected = false;
        rotation = 0.0f;
        bobOffset = 0.0f;
        type = t;
    }

    void update() {
        if (!collected) {
            rotation += 2.0f;
            if (rotation > 360.0f) rotation -= 360.0f;
            bobOffset = sin(rotation * 0.05f) * 0.3f;
        }
    }

    bool checkCollision(Vector3f playerPos, float radius) {
        if (collected) return false;
        Vector3f diff = position - playerPos;
        return diff.lengthSquared() < (radius + 0.8f) * (radius + 0.8f);
    }
};

// =============================================================================
// Lighting System
// =============================================================================
class LightingSystem {
public:
    float keyLightIntensity;
    float keyLightTime;
    Vector3f sentinelLightPos;
    float sentinelPathTime;
    int currentScene;  // 0 = Neo-Tokyo, 1 = Temple

    LightingSystem() {
        keyLightIntensity = 0.8f;
        keyLightTime = 0.0f;
        sentinelLightPos = Vector3f(0, 5, 0);
        sentinelPathTime = 0.0f;
        currentScene = 0;
    }

    void update(float deltaTime) {
        // Key Light: Oscillating intensity (sinusoidal) - brighter base
        keyLightTime += deltaTime * 0.3f;
        keyLightIntensity = 0.7f + 0.2f * sin(keyLightTime);  // Brighter range

        // Sentinel Light: Move along path
        sentinelPathTime += deltaTime * 0.5f;
        if (currentScene == 0) {
            // Neo-Tokyo: Patrolling drone path (circular)
            float radius = 15.0f;
            sentinelLightPos = Vector3f(
                cos(sentinelPathTime) * radius,
                8.0f + sin(sentinelPathTime * 2) * 2.0f,
                sin(sentinelPathTime) * radius
            );
        } else {
            // Temple: Drifting wisp path (figure-8)
            float radius = 12.0f;
            sentinelLightPos = Vector3f(
                sin(sentinelPathTime) * radius,
                6.0f + cos(sentinelPathTime * 3) * 1.5f,
                sin(sentinelPathTime * 2) * radius * 0.5f
            );
        }
    }

    void apply() {
        // Key Light (Directional) - brighter for better visibility
        float ambient[4] = {0.4f, 0.4f, 0.45f, 1.0f};
        float diffuse[4] = {keyLightIntensity * 1.2f, keyLightIntensity * 1.2f, keyLightIntensity * 1.1f, 1.0f};
        float specular[4] = {1.0f, 1.0f, 0.95f, 1.0f};
        float direction[4] = {-0.5f, -1.0f, -0.3f, 0.0f};

        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
        glLightfv(GL_LIGHT0, GL_POSITION, direction);
        glEnable(GL_LIGHT0);

        // Sentinel Light (Point) - brighter
        float pos[4] = {sentinelLightPos.x, sentinelLightPos.y, sentinelLightPos.z, 1.0f};
        float sAmbient[4] = {0.2f, 0.2f, 0.25f, 1.0f};
        float sDiffuse[4] = {0.8f, 0.8f, 0.9f, 1.0f};
        float sSpecular[4] = {1.0f, 1.0f, 1.0f, 1.0f};

        glLightfv(GL_LIGHT1, GL_AMBIENT, sAmbient);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, sDiffuse);
        glLightfv(GL_LIGHT1, GL_SPECULAR, sSpecular);
        glLightfv(GL_LIGHT1, GL_POSITION, pos);
        glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.1f);
        glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.01f);
        glEnable(GL_LIGHT1);
    }
};

// =============================================================================
// Global Game State
// =============================================================================
Player player;
Camera camera;
LightingSystem lighting;
GameState gameState = STATE_MENU;

TextureResource fallbackTexture;
TextureResource neoTokyoSurface;
TextureResource templeSurface;
TextureResource guardTexture;  // Texture for security guard
TextureResource neonSignTexture;     // Neo-Tokyo neon sign texture
TextureResource neonSignEmissive;    // Neon sign emissive/glow texture
TextureResource hieroglyphicTexture; // Egyptian hieroglyphic wall texture
TextureResource tombTexture;         // Egyptian tomb/sarcophagus texture
TextureResource columnTexture;       // Egyptian column texture
CharacterMeshData mainCharacterMesh;
OBJMesh scarabMesh;
OBJMesh cameraMesh;
OBJMesh consoleMesh;
OBJMesh guardMesh;    // Robot guard model for Neo Tokyo
OBJMesh anubisMesh;   // Anubis enemy model for Ancient Egypt
OBJMesh neonSignMesh; // Japanese neon street sign model
OBJMesh tombMesh;     // Egyptian coffin/sarcophagus model
OBJMesh columnMesh;   // Egyptian pillar/column model
bool textureSystemReady = false;
bool guardTextureLoaded = false;

// Input state
bool keys[256];
bool mouseButtons[3];
int mouseX, mouseY;
int lastMouseX, lastMouseY;
bool mouseFirstMove = true;

// Collectibles
std::vector<Collectible> crystals;
std::vector<Collectible> scarabs;

// Game state flags
bool consoleActivated = false;
bool alarmActive = false;
float alarmTime = 0.0f;
bool portalUnlocked = false;

// Animation timers
float gameTime = 0.0f;

// Pickup animation state
struct PickupAnimation {
    Vector3f position;
    float time;
    bool active;
    int type; // 0 = crystal, 1 = scarab
};
std::vector<PickupAnimation> pickupAnimations;

// Hit reaction state
float hitReactionTime = 0.0f;
bool hitReactionActive = false;

// Console activation animation state
float consoleActivationTime = 0.0f;
bool consoleActivationAnimActive = false;

// Portal activation animation state
float portalActivationTime = 0.0f;
bool portalActivationAnimActive = false;

// Security Robot struct
struct SecurityRobot {
    Vector3f position;
    Vector3f velocity;
    float yaw;              // Direction robot is facing
    float speed;            // Movement speed
    float patrolSpeed;
    float chaseSpeed;
    float fovDegrees;
    float detectDistance;
    float patrolTimer;
    float damageTimer;      // Cooldown between damage hits
    bool active;            // Whether robot is currently chasing
    bool chasing;
    
    SecurityRobot() {
        position = Vector3f(15.0f, GROUND_Y + 1.0f, 15.0f);  // Start position in Neo Tokyo
        velocity = Vector3f(0, 0, 0);
        yaw = 0.0f;
        patrolSpeed = 0.3f;      // User choice
        chaseSpeed = 0.4f;       // User choice
        speed = patrolSpeed;
        fovDegrees = 70.0f;
        detectDistance = 18.0f;
        patrolTimer = 0.0f;
        damageTimer = 0.0f;
        active = false;
        chasing = false;
    }
    
    void update(const Vector3f& playerPos, bool alarmOn, bool inNeoTokyo, float deltaTime) {
        if (!inNeoTokyo) {
            // Idle in other scenes
            active = false;
            chasing = false;
            return;
        }

        // Determine if player is in FoV
        Vector3f toPlayer = playerPos - position;
        toPlayer.y = 0;
        float dist = toPlayer.length();
        bool inSight = false;
        if (dist < detectDistance && dist > 0.001f) {
            Vector3f dir = toPlayer * (1.0f / dist);
            float yawRad = yaw * 3.14159f / 180.0f;
            Vector3f forward = Vector3f(sin(yawRad), 0, cos(yawRad));
            float dot = forward.dot(dir);
            float cosHalfFov = cosf((fovDegrees * 0.5f) * 3.14159f / 180.0f);
            inSight = dot > cosHalfFov;
        }

        if (alarmOn || inSight) {
            active = true;
            chasing = true;
            speed = chaseSpeed;
        } else {
            // Patrol
            chasing = false;
            active = true;
            speed = patrolSpeed;
        }

        if (chasing) {
            if (dist > 0.1f) {
                toPlayer = toPlayer * (1.0f / dist);
                velocity.x = toPlayer.x * speed * deltaTime * 60.0f;
                velocity.z = toPlayer.z * speed * deltaTime * 60.0f;
                yaw = atan2(toPlayer.x, toPlayer.z) * 180.0f / 3.14159f;
            }
        } else {
            // Simple circular patrol around origin
            patrolTimer += deltaTime * 0.5f;
            float radius = 12.0f;
            Vector3f target = Vector3f(cos(patrolTimer) * radius, position.y, sin(patrolTimer) * radius);
            Vector3f toTarget = target - position;
            toTarget.y = 0;
            float tDist = toTarget.length();
            if (tDist > 0.1f) {
                toTarget = toTarget * (1.0f / tDist);
                velocity.x = toTarget.x * speed * deltaTime * 60.0f;
                velocity.z = toTarget.z * speed * deltaTime * 60.0f;
                yaw = atan2(toTarget.x, toTarget.z) * 180.0f / 3.14159f;
            } else {
                velocity = Vector3f(0, 0, 0);
            }
        }

        position = position + velocity;
        position.y = GROUND_Y + 1.0f;
        if (position.x < WORLD_MIN + 2) position.x = WORLD_MIN + 2;
        if (position.x > WORLD_MAX - 2) position.x = WORLD_MAX - 2;
        if (position.z < WORLD_MIN + 2) position.z = WORLD_MIN + 2;
        if (position.z > WORLD_MAX - 2) position.z = WORLD_MAX - 2;

        if (damageTimer > 0) {
            damageTimer -= deltaTime;
        }
    }
    
    bool checkCollisionWithPlayer(const Vector3f& playerPos) {
        float dx = position.x - playerPos.x;
        float dz = position.z - playerPos.z;
        float distSq = dx * dx + dz * dz;
        return distSq < 4.0f;  // Collision radius of 2 units
    }
};

SecurityRobot securityRobot;

// =============================================================================
// Pressure Plate Trap Structure (Egypt only)
// =============================================================================
struct PressurePlateTrap {
    Vector3f position;       // Position of the plate
    bool triggered;          // Has player stepped on it
    float triggerTime;       // When was it triggered
    float trapDoorOpen;      // 0.0 = closed, 1.0 = fully open
    float spikeHeight;       // Current spike height (0 = retracted)
    bool playerInTrap;       // Is player currently in the trap pit
    float damageTimer;       // Cooldown for spike damage
    
    // Constants
    static constexpr float PLATE_SIZE = 2.5f;        // Slightly larger plate for better visibility
    static constexpr float PIT_DEPTH = 5.0f;         // Deep enough to be scary
    static constexpr float DOOR_OPEN_TIME = 0.3f;    // Quick trap door opening
    static constexpr float SPIKE_EXTEND_TIME = 0.5f; // Spike extension time
    static constexpr float SPIKE_MAX_HEIGHT = 2.0f;  // Max spike height
    static constexpr int SPIKE_DAMAGE = 5;           // Damage per spike hit
    static constexpr float DAMAGE_COOLDOWN = 0.5f;   // Damage cooldown
    
    PressurePlateTrap(Vector3f pos) {
        position = pos;
        triggered = false;
        triggerTime = 0.0f;
        trapDoorOpen = 0.0f;
        spikeHeight = 0.0f;
        playerInTrap = false;
        damageTimer = 0.0f;
    }
    
    void update(float currentTime, float deltaTime) {
        if (triggered) {
            float elapsed = currentTime - triggerTime;
            
            // Open trap door (0.3 seconds)
            if (elapsed < DOOR_OPEN_TIME) {
                trapDoorOpen = elapsed / DOOR_OPEN_TIME;
            } else {
                trapDoorOpen = 1.0f;
            }
            
            // Extend spikes after door opens (with visual telegraph delay)
            if (elapsed > DOOR_OPEN_TIME + 0.2f) {
                float spikeElapsed = elapsed - DOOR_OPEN_TIME - 0.2f;
                spikeHeight = std::min(SPIKE_MAX_HEIGHT, (spikeElapsed / SPIKE_EXTEND_TIME) * SPIKE_MAX_HEIGHT);
            }
        }
        
        if (damageTimer > 0) {
            damageTimer -= deltaTime;
        }
    }
    
    bool checkPlayerOnPlate(const Vector3f& playerPos) {
        float halfSize = PLATE_SIZE * 0.5f;
        return playerPos.x > position.x - halfSize && playerPos.x < position.x + halfSize &&
               playerPos.z > position.z - halfSize && playerPos.z < position.z + halfSize &&
               playerPos.y < position.y + 2.0f;  // Player must be at ground level
    }
    
    bool checkPlayerInPit(const Vector3f& playerPos) {
        if (!triggered || trapDoorOpen < 0.5f) return false;
        float halfSize = PLATE_SIZE * 0.5f;
        return playerPos.x > position.x - halfSize && playerPos.x < position.x + halfSize &&
               playerPos.z > position.z - halfSize && playerPos.z < position.z + halfSize &&
               playerPos.y < position.y;
    }
};

// Single pressure plate trap instance for Egypt
PressurePlateTrap templePressurePlate(Vector3f(5.0f, GROUND_Y, 0.0f));

// =============================================================================
// Moving Platform Structure (Egypt only - over fatal pit)
// =============================================================================
struct MovingPlatform {
    Vector3f basePosition;    // Center of the platform path
    float platformX;          // Current X position offset
    float speed;              // Movement speed
    float direction;          // 1.0 = moving right, -1.0 = moving left
    float travelDistance;     // How far it travels from center
    
    // Pit properties
    static constexpr float PIT_WIDTH = 6.0f;         // Narrow pit
    static constexpr float PIT_LENGTH = 8.0f;        // Length of pit
    static constexpr float PIT_DEPTH = 15.0f;        // Fatal depth
    static constexpr float PLATFORM_SIZE = 2.5f;     // Platform size
    
    MovingPlatform() {
        basePosition = Vector3f(-10.0f, GROUND_Y, 10.0f);  // Platform location
        platformX = 0.0f;
        speed = 3.0f;         // Units per second
        direction = 1.0f;
        travelDistance = 2.0f; // Travels 2 units left/right from center
    }
    
    void update(float deltaTime) {
        platformX += direction * speed * deltaTime;
        
        // Reverse direction at ends
        if (platformX > travelDistance) {
            platformX = travelDistance;
            direction = -1.0f;
        } else if (platformX < -travelDistance) {
            platformX = -travelDistance;
            direction = 1.0f;
        }
    }
    
    Vector3f getPlatformPosition() const {
        return Vector3f(basePosition.x + platformX, basePosition.y + 0.5f, basePosition.z);
    }
    
    bool checkPlayerOnPlatform(const Vector3f& playerPos) const {
        Vector3f platPos = getPlatformPosition();
        float halfSize = PLATFORM_SIZE * 0.5f;
        float dy = playerPos.y - (platPos.y + 0.3f);  // Standing on top
        return playerPos.x > platPos.x - halfSize && playerPos.x < platPos.x + halfSize &&
               playerPos.z > platPos.z - halfSize && playerPos.z < platPos.z + halfSize &&
               dy > -0.5f && dy < 1.5f;
    }
    
    bool checkPlayerInPit(const Vector3f& playerPos) const {
        float halfWidth = PIT_WIDTH * 0.5f;
        float halfLength = PIT_LENGTH * 0.5f;
        return playerPos.x > basePosition.x - halfWidth && playerPos.x < basePosition.x + halfWidth &&
               playerPos.z > basePosition.z - halfLength && playerPos.z < basePosition.z + halfLength &&
               playerPos.y < basePosition.y - 1.0f;
    }
};

MovingPlatform templeMovingPlatform;

// =============================================================================
// Guardian Statue Patrol Structure (Egypt only)
// =============================================================================
struct GuardianPatrol {
    Vector3f homePosition;    // Starting/center position
    Vector3f position;        // Current position
    float patrolOffset;       // Current offset along patrol path
    float patrolDirection;    // 1.0 = forward, -1.0 = backward
    float patrolDistance;     // How far to patrol (5 units)
    float patrolSpeed;        // Same speed as current chase
    float chaseSpeed;         // Chase speed when player near
    float yaw;                // Current facing direction
    float detectRadius;       // Radius to detect and chase player
    bool chasing;             // Currently chasing player
    float damageTimer;        // Damage cooldown
    
    GuardianPatrol() {
        homePosition = Vector3f(15.0f, GROUND_Y + 1.0f, 15.0f);
        position = homePosition;
        patrolOffset = 0.0f;
        patrolDirection = 1.0f;
        patrolDistance = 5.0f;    // Patrol 5 units back and forth
        patrolSpeed = 0.15f;      // Slow patrol
        chaseSpeed = 0.3f;        // Same as current robot chase
        yaw = 0.0f;
        detectRadius = 6.0f;      // Small radius for chase trigger
        chasing = false;
        damageTimer = 0.0f;
    }
    
    void update(const Vector3f& playerPos, float deltaTime) {
        // Check if player is within chase radius
        Vector3f toPlayer = playerPos - position;
        toPlayer.y = 0;
        float dist = toPlayer.length();
        
        if (dist < detectRadius && dist > 0.1f) {
            // Chase player
            chasing = true;
            Vector3f dir = toPlayer * (1.0f / dist);
            position.x += dir.x * chaseSpeed * deltaTime * 60.0f;
            position.z += dir.z * chaseSpeed * deltaTime * 60.0f;
            yaw = RAD2DEG(atan2(dir.x, dir.z));
        } else {
            // Patrol back and forth along Z axis
            chasing = false;
            patrolOffset += patrolDirection * patrolSpeed * deltaTime * 60.0f;
            
            if (patrolOffset > patrolDistance) {
                patrolOffset = patrolDistance;
                patrolDirection = -1.0f;
            } else if (patrolOffset < -patrolDistance) {
                patrolOffset = -patrolDistance;
                patrolDirection = 1.0f;
            }
            
            position = Vector3f(homePosition.x, homePosition.y, homePosition.z + patrolOffset);
            yaw = patrolDirection > 0 ? 0.0f : 180.0f;  // Face patrol direction
        }
        
        if (damageTimer > 0) {
            damageTimer -= deltaTime;
        }
    }
    
    bool checkCollisionWithPlayer(const Vector3f& playerPos) {
        float dx = position.x - playerPos.x;
        float dz = position.z - playerPos.z;
        float distSq = dx * dx + dz * dz;
        return distSq < 4.0f;  // Collision radius of 2 units
    }
};

GuardianPatrol templeGuardian;

// =============================================================================
// Texture & Model Loading Helpers
// =============================================================================

TextureResource createSolidTexture(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) {
    TextureResource tex;
    glGenTextures(1, &tex.id);
    glBindTexture(GL_TEXTURE_2D, tex.id);
    unsigned char data[4] = {r, g, b, a};
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    tex.width = 1;
    tex.height = 1;
    tex.source = "solid_color";
    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

TextureResource loadTextureFromFile(const std::string& path) {
    TextureResource tex;
    int width, height, channels;
    stbi_uc* pixels = stbi_load(path.c_str(), &width, &height, &channels, STBI_rgb_alpha);
    if (!pixels) {
        printf("Failed to load texture: %s\n", path.c_str());
        return tex;
    }

    glGenTextures(1, &tex.id);
    glBindTexture(GL_TEXTURE_2D, tex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    tex.width = width;
    tex.height = height;
    tex.source = path;

    stbi_image_free(pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

TextureResource textureFromTinyImage(const tinygltf::Image& image) {
    TextureResource tex;
    if (image.image.empty()) {
        return tex;
    }

    GLint format = GL_RGBA;
#ifdef GL_LUMINANCE
    if (image.component == 1) format = GL_LUMINANCE;
    else if (image.component == 2) format = GL_LUMINANCE_ALPHA;
    else
#endif
    if (image.component == 3) format = GL_RGB;

    glGenTextures(1, &tex.id);
    glBindTexture(GL_TEXTURE_2D, tex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, format, image.width, image.height, format, GL_UNSIGNED_BYTE, image.image.data());

    tex.width = image.width;
    tex.height = image.height;
    tex.source = image.uri.empty() ? "embedded_texture" : image.uri;

    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

bool initializeTextureAssets() {
    if (textureSystemReady) return true;
    stbi_set_flip_vertically_on_load(true);

    fallbackTexture = createSolidTexture(255, 255, 255);
    neoTokyoSurface = loadTextureFromFile(MODERN_TOKYO_BASE);
    templeSurface = loadTextureFromFile(ANCIENT_EGYPT_BASE);

    if (!neoTokyoSurface.valid()) neoTokyoSurface = fallbackTexture;
    if (!templeSurface.valid()) templeSurface = fallbackTexture;

    // Load new textures for Neo-Tokyo and Egypt enhancements
    neonSignTexture = loadTextureFromFile(NEON_SIGN_TEXTURE_PATH);
    neonSignEmissive = loadTextureFromFile(NEON_SIGN_EMISSIVE_PATH);
    hieroglyphicTexture = loadTextureFromFile(HIEROGLYPHIC_TEXTURE_PATH);
    tombTexture = loadTextureFromFile(TOMB_TEXTURE_PATH);
    columnTexture = loadTextureFromFile(COLUMN_TEXTURE_PATH);
    
    if (!neonSignTexture.valid()) neonSignTexture = fallbackTexture;
    if (!neonSignEmissive.valid()) neonSignEmissive = fallbackTexture;
    if (!hieroglyphicTexture.valid()) hieroglyphicTexture = fallbackTexture;
    if (!tombTexture.valid()) tombTexture = fallbackTexture;
    if (!columnTexture.valid()) columnTexture = fallbackTexture;

    textureSystemReady = true;
    return true;
}

// =============================================================================
// 4x4 Matrix Structure for Node Transformations
// =============================================================================
// Column-major order (OpenGL style): m[col][row]
// Used for GLTF node hierarchy transformations
struct Matrix4x4 {
    float m[16];  // Column-major: [0-3]=col0, [4-7]=col1, [8-11]=col2, [12-15]=col3
    
    // Default constructor: Identity matrix
    Matrix4x4() {
        // Initialize to identity matrix
        for (int i = 0; i < 16; i++) m[i] = 0.0f;
        m[0] = m[5] = m[10] = m[15] = 1.0f;  // Diagonal = 1
    }
    
    // Constructor from array (column-major)
    Matrix4x4(const float* data) {
        for (int i = 0; i < 16; i++) m[i] = data[i];
    }
    
    // Constructor from double array (GLTF uses doubles)
    Matrix4x4(const std::vector<double>& data) {
        if (data.size() >= 16) {
            for (int i = 0; i < 16; i++) m[i] = static_cast<float>(data[i]);
        } else {
            // Default to identity if data is invalid
            for (int i = 0; i < 16; i++) m[i] = 0.0f;
            m[0] = m[5] = m[10] = m[15] = 1.0f;
        }
    }
    
    // Create identity matrix
    static Matrix4x4 identity() {
        return Matrix4x4();
    }
    
    // Create translation matrix
    static Matrix4x4 translate(float x, float y, float z) {
        Matrix4x4 result;
        result.m[12] = x;
        result.m[13] = y;
        result.m[14] = z;
        return result;
    }
    
    // Create scale matrix
    static Matrix4x4 scale(float x, float y, float z) {
        Matrix4x4 result;
        result.m[0] = x;
        result.m[5] = y;
        result.m[10] = z;
        return result;
    }
    
    // Create rotation matrix from quaternion (x, y, z, w)
    static Matrix4x4 fromQuaternion(float qx, float qy, float qz, float qw) {
        Matrix4x4 result;
        
        float xx = qx * qx, yy = qy * qy, zz = qz * qz;
        float xy = qx * qy, xz = qx * qz, yz = qy * qz;
        float wx = qw * qx, wy = qw * qy, wz = qw * qz;
        
        result.m[0]  = 1.0f - 2.0f * (yy + zz);
        result.m[1]  = 2.0f * (xy + wz);
        result.m[2]  = 2.0f * (xz - wy);
        result.m[3]  = 0.0f;
        
        result.m[4]  = 2.0f * (xy - wz);
        result.m[5]  = 1.0f - 2.0f * (xx + zz);
        result.m[6]  = 2.0f * (yz + wx);
        result.m[7]  = 0.0f;
        
        result.m[8]  = 2.0f * (xz + wy);
        result.m[9]  = 2.0f * (yz - wx);
        result.m[10] = 1.0f - 2.0f * (xx + yy);
        result.m[11] = 0.0f;
        
        result.m[12] = 0.0f;
        result.m[13] = 0.0f;
        result.m[14] = 0.0f;
        result.m[15] = 1.0f;
        
        return result;
    }
    
    // Matrix multiplication: this * other
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        for (int i = 0; i < 16; i++) result.m[i] = 0.0f;
        
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                for (int k = 0; k < 4; k++) {
                    result.m[col * 4 + row] += m[k * 4 + row] * other.m[col * 4 + k];
                }
            }
        }
        return result;
    }
    
    // Transform a 3D point (assumes w=1)
    Vector3f transformPoint(const Vector3f& p) const {
        float x = m[0] * p.x + m[4] * p.y + m[8]  * p.z + m[12];
        float y = m[1] * p.x + m[5] * p.y + m[9]  * p.z + m[13];
        float z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
        float w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
        if (fabs(w) > 0.0001f) {
            x /= w; y /= w; z /= w;
        }
        return Vector3f(x, y, z);
    }
    
    // Transform a 3D normal (ignores translation)
    Vector3f transformNormal(const Vector3f& n) const {
        float x = m[0] * n.x + m[4] * n.y + m[8]  * n.z;
        float y = m[1] * n.x + m[5] * n.y + m[9]  * n.z;
        float z = m[2] * n.x + m[6] * n.y + m[10] * n.z;
        return Vector3f(x, y, z).unit();
    }
    
    // Apply this matrix to OpenGL (uses glMultMatrixf)
    void applyToGL() const {
        glMultMatrixf(m);
    }
};

// Global model reference for traverseNode (set during loading)
static tinygltf::Model* g_currentModel = nullptr;

// Forward declarations for GLTF data reading functions
bool readAccessorData(const tinygltf::Model& model, const tinygltf::Accessor& accessor, std::vector<float>& outData, int components);
bool readIndexData(const tinygltf::Model& model, const tinygltf::Accessor& accessor, std::vector<unsigned int>& outData);

// =============================================================================
// traverseNode - Recursively process GLTF node hierarchy
// =============================================================================
// Parameters:
//   nodeIndex  - Index of the node in the GLTF model's nodes array
//   parentMatrix - The accumulated transformation matrix from parent nodes
//                  (defaults to identity matrix if not specified)
//
// This function:
// 1. Computes the node's local transformation from TRS or matrix
// 2. Multiplies with parent matrix to get world transform
// 3. If node has a mesh, transforms and stores the mesh data
// 4. Recursively processes all child nodes
// =============================================================================
void traverseNode(int nodeIndex, Matrix4x4 parentMatrix = Matrix4x4::identity()) {
    if (g_currentModel == nullptr || nodeIndex < 0 || nodeIndex >= (int)g_currentModel->nodes.size()) {
        return;
    }
    
    const tinygltf::Node& node = g_currentModel->nodes[nodeIndex];
    
    // =============================================================================
    // Step 1: Compute local transformation matrix for this node
    // =============================================================================
    Matrix4x4 localMatrix = Matrix4x4::identity();
    
    if (!node.matrix.empty() && node.matrix.size() == 16) {
        // Node has explicit matrix - use it directly
        localMatrix = Matrix4x4(node.matrix);
    } else {
        // Build matrix from Translation, Rotation, Scale (TRS)
        // Order: localMatrix = T * R * S
        
        // Scale
        Matrix4x4 S = Matrix4x4::identity();
        if (node.scale.size() == 3) {
            S = Matrix4x4::scale(
                static_cast<float>(node.scale[0]),
                static_cast<float>(node.scale[1]),
                static_cast<float>(node.scale[2])
            );
        }
        
        // Rotation (quaternion: x, y, z, w)
        Matrix4x4 R = Matrix4x4::identity();
        if (node.rotation.size() == 4) {
            R = Matrix4x4::fromQuaternion(
                static_cast<float>(node.rotation[0]),
                static_cast<float>(node.rotation[1]),
                static_cast<float>(node.rotation[2]),
                static_cast<float>(node.rotation[3])
            );
        }
        
        // Translation
        Matrix4x4 T = Matrix4x4::identity();
        if (node.translation.size() == 3) {
            T = Matrix4x4::translate(
                static_cast<float>(node.translation[0]),
                static_cast<float>(node.translation[1]),
                static_cast<float>(node.translation[2])
            );
        }
        
        // Combine: T * R * S
        localMatrix = T * R * S;
    }
    
    // =============================================================================
    // Step 2: Compute world transformation (parent * local)
    // =============================================================================
    Matrix4x4 worldMatrix = parentMatrix * localMatrix;
    
    // =============================================================================
    // Step 3: If this node has a mesh, process it with the world transform
    // =============================================================================
    if (node.mesh >= 0 && node.mesh < (int)g_currentModel->meshes.size()) {
        const tinygltf::Mesh& mesh = g_currentModel->meshes[node.mesh];
        
        // Process each primitive in the mesh
        for (size_t primIdx = 0; primIdx < mesh.primitives.size(); primIdx++) {
            const tinygltf::Primitive& primitive = mesh.primitives[primIdx];
            
            // Get position data
            auto posIt = primitive.attributes.find("POSITION");
            if (posIt == primitive.attributes.end()) continue;
            
            const tinygltf::Accessor& posAccessor = g_currentModel->accessors[posIt->second];
            std::vector<float> positions;
            if (!readAccessorData(*g_currentModel, posAccessor, positions, 3)) continue;
            
            // Transform positions by world matrix and add to main mesh
            size_t baseVertex = mainCharacterMesh.positions.size() / 3;
            for (size_t i = 0; i < positions.size(); i += 3) {
                Vector3f pos(positions[i], positions[i + 1], positions[i + 2]);
                Vector3f transformed = worldMatrix.transformPoint(pos);
                mainCharacterMesh.positions.push_back(transformed.x);
                mainCharacterMesh.positions.push_back(transformed.y);
                mainCharacterMesh.positions.push_back(transformed.z);
            }
            
            // Get and transform normals
            auto normalIt = primitive.attributes.find("NORMAL");
            if (normalIt != primitive.attributes.end()) {
                std::vector<float> normals;
                if (readAccessorData(*g_currentModel, g_currentModel->accessors[normalIt->second], normals, 3)) {
                    for (size_t i = 0; i < normals.size(); i += 3) {
                        Vector3f normal(normals[i], normals[i + 1], normals[i + 2]);
                        Vector3f transformed = worldMatrix.transformNormal(normal);
                        mainCharacterMesh.normals.push_back(transformed.x);
                        mainCharacterMesh.normals.push_back(transformed.y);
                        mainCharacterMesh.normals.push_back(transformed.z);
                    }
                }
            }
            
            // Get texture coordinates (no transformation needed)
            auto uvIt = primitive.attributes.find("TEXCOORD_0");
            if (uvIt != primitive.attributes.end()) {
                std::vector<float> uvs;
                if (readAccessorData(*g_currentModel, g_currentModel->accessors[uvIt->second], uvs, 2)) {
                    for (size_t i = 0; i < uvs.size(); i++) {
                        mainCharacterMesh.texcoords.push_back(uvs[i]);
                    }
                }
            }
            
            // Get indices and offset by base vertex
            if (primitive.indices >= 0) {
                const tinygltf::Accessor& indexAccessor = g_currentModel->accessors[primitive.indices];
                std::vector<unsigned int> indices;
                if (readIndexData(*g_currentModel, indexAccessor, indices)) {
                    for (size_t i = 0; i < indices.size(); i++) {
                        mainCharacterMesh.indices.push_back(static_cast<unsigned int>(baseVertex + indices[i]));
                    }
                }
            } else {
                // No indices - create sequential indices
                size_t vertexCount = positions.size() / 3;
                for (size_t i = 0; i < vertexCount; i++) {
                    mainCharacterMesh.indices.push_back(static_cast<unsigned int>(baseVertex + i));
                }
            }
            
            // Load texture from material (only for first primitive with texture)
            if (!mainCharacterMesh.albedo.valid() && primitive.material >= 0 && 
                primitive.material < (int)g_currentModel->materials.size()) {
                const auto& material = g_currentModel->materials[primitive.material];
                if (material.pbrMetallicRoughness.baseColorTexture.index >= 0) {
                    int texIdx = material.pbrMetallicRoughness.baseColorTexture.index;
                    if (texIdx < (int)g_currentModel->textures.size()) {
                        const tinygltf::Texture& tex = g_currentModel->textures[texIdx];
                        if (tex.source >= 0 && tex.source < (int)g_currentModel->images.size()) {
                            mainCharacterMesh.albedo = textureFromTinyImage(g_currentModel->images[tex.source]);
                        }
                    }
                }
            }
        }
    }
    
    // =============================================================================
    // Step 4: Recursively process all child nodes
    // =============================================================================
    for (size_t i = 0; i < node.children.size(); i++) {
        traverseNode(node.children[i], worldMatrix);
    }
}

bool readAccessorData(const tinygltf::Model& model, const tinygltf::Accessor& accessor, std::vector<float>& outData, int components) {
    if (accessor.bufferView < 0) return false;
    const tinygltf::BufferView& view = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer& buffer = model.buffers[view.buffer];

    const unsigned char* dataPtr = buffer.data.data() + view.byteOffset + accessor.byteOffset;
    size_t stride = accessor.ByteStride(view);
    if (stride == 0) {
        stride = components * sizeof(float);
    }

    outData.resize(accessor.count * components);
    for (size_t i = 0; i < accessor.count; ++i) {
        memcpy(&outData[i * components], dataPtr + stride * i, components * sizeof(float));
    }
    return true;
}

bool readIndexData(const tinygltf::Model& model, const tinygltf::Accessor& accessor, std::vector<unsigned int>& outData) {
    if (accessor.bufferView < 0) return false;
    const tinygltf::BufferView& view = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer& buffer = model.buffers[view.buffer];
    const unsigned char* dataPtr = buffer.data.data() + view.byteOffset + accessor.byteOffset;

    outData.resize(accessor.count);
    for (size_t i = 0; i < accessor.count; ++i) {
        switch (accessor.componentType) {
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                outData[i] = reinterpret_cast<const unsigned short*>(dataPtr)[i];
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                outData[i] = reinterpret_cast<const unsigned char*>(dataPtr)[i];
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                outData[i] = reinterpret_cast<const unsigned int*>(dataPtr)[i];
                break;
            default:
                return false;
        }
    }
    return true;
}

// =============================================================================
// OBJ File Loader for Character Model
// =============================================================================
bool loadMainCharacterModel() {
    FILE* file = fopen(CHARACTER_MODEL_PATH, "r");
    if (!file) {
        printf("Failed to open OBJ file: %s\n", CHARACTER_MODEL_PATH);
        return false;
    }
    
    // Clear any existing mesh data
    mainCharacterMesh.positions.clear();
    mainCharacterMesh.normals.clear();
    mainCharacterMesh.texcoords.clear();
    mainCharacterMesh.indices.clear();
    
    // Temporary storage for OBJ data
    std::vector<float> tempPositions;  // v: x, y, z
    std::vector<float> tempTexcoords;  // vt: u, v
    std::vector<float> tempNormals;    // vn: nx, ny, nz
    
    // Final vertex data (expanded from faces)
    std::vector<float> finalPositions;
    std::vector<float> finalTexcoords;
    std::vector<float> finalNormals;
    
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        
        // Vertex position: v x y z
        if (line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            if (sscanf(line, "v %f %f %f", &x, &y, &z) == 3) {
                tempPositions.push_back(x);
                tempPositions.push_back(y);
                tempPositions.push_back(z);
            }
        }
        // Texture coordinate: vt u v
        else if (line[0] == 'v' && line[1] == 't') {
            float u, v;
            if (sscanf(line, "vt %f %f", &u, &v) == 2) {
                tempTexcoords.push_back(u);
                tempTexcoords.push_back(v);
            }
        }
        // Vertex normal: vn nx ny nz
        else if (line[0] == 'v' && line[1] == 'n') {
            float nx, ny, nz;
            if (sscanf(line, "vn %f %f %f", &nx, &ny, &nz) == 3) {
                tempNormals.push_back(nx);
                tempNormals.push_back(ny);
                tempNormals.push_back(nz);
            }
        }
        // Face: f v/vt/vn v/vt/vn v/vt/vn (triangles)
        // Can also be: f v//vn, f v/vt, f v
        else if (line[0] == 'f' && line[1] == ' ') {
            int v[4], vt[4], vn[4];
            int numVerts = 0;
            
            // Initialize to -1 (not present)
            for (int i = 0; i < 4; i++) { v[i] = vt[i] = vn[i] = -1; }
            
            // Try different face formats
            // Format: f v/vt/vn v/vt/vn v/vt/vn [v/vt/vn]
            int matches = sscanf(line, "f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",
                &v[0], &vt[0], &vn[0], &v[1], &vt[1], &vn[1], 
                &v[2], &vt[2], &vn[2], &v[3], &vt[3], &vn[3]);
            
            if (matches >= 9) {
                numVerts = matches / 3;
            } else {
                // Try format: f v//vn v//vn v//vn
                matches = sscanf(line, "f %d//%d %d//%d %d//%d %d//%d",
                    &v[0], &vn[0], &v[1], &vn[1], &v[2], &vn[2], &v[3], &vn[3]);
                if (matches >= 6) {
                    numVerts = matches / 2;
                } else {
                    // Try format: f v/vt v/vt v/vt
                    matches = sscanf(line, "f %d/%d %d/%d %d/%d %d/%d",
                        &v[0], &vt[0], &v[1], &vt[1], &v[2], &vt[2], &v[3], &vt[3]);
                    if (matches >= 6) {
                        numVerts = matches / 2;
                    } else {
                        // Try format: f v v v
                        matches = sscanf(line, "f %d %d %d %d", &v[0], &v[1], &v[2], &v[3]);
                        if (matches >= 3) {
                            numVerts = matches;
                        }
                    }
                }
            }
            
            // Process triangle(s) from face
            // First triangle: v[0], v[1], v[2]
            if (numVerts >= 3) {
                for (int i = 0; i < 3; i++) {
                    int posIdx = v[i] - 1;  // OBJ indices are 1-based
                    int texIdx = vt[i] - 1;
                    int normIdx = vn[i] - 1;
                    
                    // Position (required)
                    if (posIdx >= 0 && posIdx * 3 + 2 < (int)tempPositions.size()) {
                        finalPositions.push_back(tempPositions[posIdx * 3]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 1]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 2]);
                    } else {
                        finalPositions.push_back(0); finalPositions.push_back(0); finalPositions.push_back(0);
                    }
                    
                    // Texture coordinate (optional)
                    if (texIdx >= 0 && texIdx * 2 + 1 < (int)tempTexcoords.size()) {
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2]);
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2 + 1]);
                    } else {
                        finalTexcoords.push_back(0); finalTexcoords.push_back(0);
                    }
                    
                    // Normal (optional)
                    if (normIdx >= 0 && normIdx * 3 + 2 < (int)tempNormals.size()) {
                        finalNormals.push_back(tempNormals[normIdx * 3]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 1]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 2]);
                    } else {
                        finalNormals.push_back(0); finalNormals.push_back(1); finalNormals.push_back(0);
                    }
                }
            }
            
            // Second triangle for quad: v[0], v[2], v[3]
            if (numVerts >= 4) {
                int quadIndices[3] = {0, 2, 3};
                for (int j = 0; j < 3; j++) {
                    int i = quadIndices[j];
                    int posIdx = v[i] - 1;
                    int texIdx = vt[i] - 1;
                    int normIdx = vn[i] - 1;
                    
                    if (posIdx >= 0 && posIdx * 3 + 2 < (int)tempPositions.size()) {
                        finalPositions.push_back(tempPositions[posIdx * 3]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 1]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 2]);
                    } else {
                        finalPositions.push_back(0); finalPositions.push_back(0); finalPositions.push_back(0);
                    }
                    
                    if (texIdx >= 0 && texIdx * 2 + 1 < (int)tempTexcoords.size()) {
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2]);
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2 + 1]);
                    } else {
                        finalTexcoords.push_back(0); finalTexcoords.push_back(0);
                    }
                    
                    if (normIdx >= 0 && normIdx * 3 + 2 < (int)tempNormals.size()) {
                        finalNormals.push_back(tempNormals[normIdx * 3]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 1]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 2]);
                    } else {
                        finalNormals.push_back(0); finalNormals.push_back(1); finalNormals.push_back(0);
                    }
                }
            }
        }
    }
    
    fclose(file);
    
    // Copy to main mesh
    mainCharacterMesh.positions = finalPositions;
    mainCharacterMesh.normals = finalNormals;
    mainCharacterMesh.texcoords = finalTexcoords;
    
    // Create sequential indices (vertices are already expanded)
    mainCharacterMesh.indices.resize(mainCharacterMesh.positions.size() / 3);
    for (size_t i = 0; i < mainCharacterMesh.indices.size(); i++) {
        mainCharacterMesh.indices[i] = static_cast<unsigned int>(i);
    }
    
    // Check if we got any mesh data
    if (mainCharacterMesh.positions.empty()) {
        printf("No mesh data was loaded from OBJ file.\n");
        return false;
    }

    // Compute bounds
    mainCharacterMesh.boundsMin = Vector3f(mainCharacterMesh.positions[0], mainCharacterMesh.positions[1], mainCharacterMesh.positions[2]);
    mainCharacterMesh.boundsMax = mainCharacterMesh.boundsMin;
    for (size_t i = 0; i < mainCharacterMesh.positions.size(); i += 3) {
        float x = mainCharacterMesh.positions[i];
        float y = mainCharacterMesh.positions[i + 1];
        float z = mainCharacterMesh.positions[i + 2];
        mainCharacterMesh.boundsMin.x = std::min(mainCharacterMesh.boundsMin.x, x);
        mainCharacterMesh.boundsMin.y = std::min(mainCharacterMesh.boundsMin.y, y);
        mainCharacterMesh.boundsMin.z = std::min(mainCharacterMesh.boundsMin.z, z);
        mainCharacterMesh.boundsMax.x = std::max(mainCharacterMesh.boundsMax.x, x);
        mainCharacterMesh.boundsMax.y = std::max(mainCharacterMesh.boundsMax.y, y);
        mainCharacterMesh.boundsMax.z = std::max(mainCharacterMesh.boundsMax.z, z);
    }
    
    float height = std::max(0.001f, mainCharacterMesh.boundsMax.y - mainCharacterMesh.boundsMin.y);
    mainCharacterMesh.scale = PLAYER_HEIGHT / height;
    mainCharacterMesh.boundsCenter = Vector3f(
        (mainCharacterMesh.boundsMin.x + mainCharacterMesh.boundsMax.x) * 0.5f,
        (mainCharacterMesh.boundsMin.y + mainCharacterMesh.boundsMax.y) * 0.5f,
        (mainCharacterMesh.boundsMin.z + mainCharacterMesh.boundsMax.z) * 0.5f
    );

    // Load character texture
    mainCharacterMesh.albedo = loadTextureFromFile(CHARACTER_TEXTURE_PATH);
    if (!mainCharacterMesh.albedo.valid()) {
        printf("Warning: Could not load character texture, using fallback.\n");
        mainCharacterMesh.albedo = fallbackTexture;
    }

    mainCharacterMesh.loaded = true;
    printf("Loaded OBJ character mesh with %zu vertices and %zu triangles.\n",
           mainCharacterMesh.positions.size() / 3,
           mainCharacterMesh.indices.size() / 3);
    printf("Model bounds: min(%.2f, %.2f, %.2f) max(%.2f, %.2f, %.2f)\n",
           mainCharacterMesh.boundsMin.x, mainCharacterMesh.boundsMin.y, mainCharacterMesh.boundsMin.z,
           mainCharacterMesh.boundsMax.x, mainCharacterMesh.boundsMax.y, mainCharacterMesh.boundsMax.z);
    return true;
}

// =============================================================================
// Generic OBJ Loader for Props
// =============================================================================
bool loadOBJMesh(const char* modelPath, const char* texturePath, OBJMesh& mesh) {
    FILE* file = fopen(modelPath, "r");
    if (!file) {
        printf("Failed to open OBJ file: %s\n", modelPath);
        return false;
    }
    
    mesh.positions.clear();
    mesh.normals.clear();
    mesh.texcoords.clear();
    mesh.indices.clear();
    
    std::vector<float> tempPositions, tempTexcoords, tempNormals;
    std::vector<float> finalPositions, finalTexcoords, finalNormals;
    
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        
        if (line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            if (sscanf(line, "v %f %f %f", &x, &y, &z) == 3) {
                tempPositions.push_back(x);
                tempPositions.push_back(y);
                tempPositions.push_back(z);
            }
        }
        else if (line[0] == 'v' && line[1] == 't') {
            float u, v;
            if (sscanf(line, "vt %f %f", &u, &v) == 2) {
                tempTexcoords.push_back(u);
                tempTexcoords.push_back(v);
            }
        }
        else if (line[0] == 'v' && line[1] == 'n') {
            float nx, ny, nz;
            if (sscanf(line, "vn %f %f %f", &nx, &ny, &nz) == 3) {
                tempNormals.push_back(nx);
                tempNormals.push_back(ny);
                tempNormals.push_back(nz);
            }
        }
        else if (line[0] == 'f' && line[1] == ' ') {
            int v[4], vt[4], vn[4];
            int numVerts = 0;
            for (int i = 0; i < 4; i++) { v[i] = vt[i] = vn[i] = -1; }
            
            int matches = sscanf(line, "f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",
                &v[0], &vt[0], &vn[0], &v[1], &vt[1], &vn[1], 
                &v[2], &vt[2], &vn[2], &v[3], &vt[3], &vn[3]);
            
            if (matches >= 9) {
                numVerts = matches / 3;
            } else {
                matches = sscanf(line, "f %d//%d %d//%d %d//%d %d//%d",
                    &v[0], &vn[0], &v[1], &vn[1], &v[2], &vn[2], &v[3], &vn[3]);
                if (matches >= 6) {
                    numVerts = matches / 2;
                } else {
                    matches = sscanf(line, "f %d/%d %d/%d %d/%d %d/%d",
                        &v[0], &vt[0], &v[1], &vt[1], &v[2], &vt[2], &v[3], &vt[3]);
                    if (matches >= 6) {
                        numVerts = matches / 2;
                    } else {
                        matches = sscanf(line, "f %d %d %d %d", &v[0], &v[1], &v[2], &v[3]);
                        if (matches >= 3) numVerts = matches;
                    }
                }
            }
            
            // First triangle
            if (numVerts >= 3) {
                for (int i = 0; i < 3; i++) {
                    int posIdx = v[i] - 1, texIdx = vt[i] - 1, normIdx = vn[i] - 1;
                    if (posIdx >= 0 && posIdx * 3 + 2 < (int)tempPositions.size()) {
                        finalPositions.push_back(tempPositions[posIdx * 3]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 1]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 2]);
                    } else { finalPositions.push_back(0); finalPositions.push_back(0); finalPositions.push_back(0); }
                    
                    if (texIdx >= 0 && texIdx * 2 + 1 < (int)tempTexcoords.size()) {
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2]);
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2 + 1]);
                    } else { finalTexcoords.push_back(0); finalTexcoords.push_back(0); }
                    
                    if (normIdx >= 0 && normIdx * 3 + 2 < (int)tempNormals.size()) {
                        finalNormals.push_back(tempNormals[normIdx * 3]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 1]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 2]);
                    } else { finalNormals.push_back(0); finalNormals.push_back(1); finalNormals.push_back(0); }
                }
            }
            
            // Second triangle for quad
            if (numVerts >= 4) {
                int quadIndices[3] = {0, 2, 3};
                for (int j = 0; j < 3; j++) {
                    int i = quadIndices[j];
                    int posIdx = v[i] - 1, texIdx = vt[i] - 1, normIdx = vn[i] - 1;
                    if (posIdx >= 0 && posIdx * 3 + 2 < (int)tempPositions.size()) {
                        finalPositions.push_back(tempPositions[posIdx * 3]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 1]);
                        finalPositions.push_back(tempPositions[posIdx * 3 + 2]);
                    } else { finalPositions.push_back(0); finalPositions.push_back(0); finalPositions.push_back(0); }
                    
                    if (texIdx >= 0 && texIdx * 2 + 1 < (int)tempTexcoords.size()) {
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2]);
                        finalTexcoords.push_back(tempTexcoords[texIdx * 2 + 1]);
                    } else { finalTexcoords.push_back(0); finalTexcoords.push_back(0); }
                    
                    if (normIdx >= 0 && normIdx * 3 + 2 < (int)tempNormals.size()) {
                        finalNormals.push_back(tempNormals[normIdx * 3]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 1]);
                        finalNormals.push_back(tempNormals[normIdx * 3 + 2]);
                    } else { finalNormals.push_back(0); finalNormals.push_back(1); finalNormals.push_back(0); }
                }
            }
        }
    }
    fclose(file);
    
    mesh.positions = finalPositions;
    mesh.normals = finalNormals;
    mesh.texcoords = finalTexcoords;
    
    mesh.indices.resize(mesh.positions.size() / 3);
    for (size_t i = 0; i < mesh.indices.size(); i++) {
        mesh.indices[i] = static_cast<unsigned int>(i);
    }
    
    if (mesh.positions.empty()) {
        printf("No mesh data loaded from: %s\n", modelPath);
        return false;
    }
    
    mesh.computeBounds();
    
    // Load texture if provided
    if (texturePath && strlen(texturePath) > 0) {
        mesh.albedo = loadTextureFromFile(texturePath);
    }
    if (!mesh.albedo.valid()) {
        mesh.albedo = fallbackTexture;
    }
    
    mesh.loaded = true;
    printf("Loaded OBJ: %s (%zu verts, %zu tris)\n", modelPath,
           mesh.positions.size() / 3, mesh.indices.size() / 3);
    return true;
}

// Draw an OBJMesh at a given position with scale and rotation
void drawOBJMesh(const OBJMesh& mesh, Vector3f pos, float rotY = 0.0f, float scale = 1.0f) {
    if (!mesh.loaded || mesh.positions.empty()) return;
    
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotY, 0, 1, 0);
    glScalef(scale, scale, scale);
    
    // Center the mesh
    glTranslatef(-mesh.boundsCenter.x, -mesh.boundsMin.y, -mesh.boundsCenter.z);
    
    bool hasUV = !mesh.texcoords.empty() && mesh.texcoords.size() / 2 >= mesh.positions.size() / 3;
    bool hasNormals = !mesh.normals.empty();
    
    if (mesh.albedo.valid()) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, mesh.albedo.id);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D);
    }
    
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < mesh.indices.size(); i++) {
        unsigned int idx = mesh.indices[i];
        if (hasUV && idx * 2 + 1 < mesh.texcoords.size()) {
            glTexCoord2f(mesh.texcoords[idx * 2], mesh.texcoords[idx * 2 + 1]);
        }
        if (hasNormals && idx * 3 + 2 < mesh.normals.size()) {
            glNormal3f(mesh.normals[idx * 3], mesh.normals[idx * 3 + 1], mesh.normals[idx * 3 + 2]);
        }
        if (idx * 3 + 2 < mesh.positions.size()) {
            glVertex3f(mesh.positions[idx * 3], mesh.positions[idx * 3 + 1], mesh.positions[idx * 3 + 2]);
        }
    }
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
}

// Load all prop models
bool loadPropModels() {
    printf("Loading prop models...\n");
    
    bool success = true;
    
    if (!loadOBJMesh(SCARAB_MODEL_PATH, SCARAB_TEXTURE_PATH, scarabMesh)) {
        printf("Warning: Failed to load scarab model\n");
        success = false;
    }
    
    if (!loadOBJMesh(CAMERA_MODEL_PATH, CAMERA_TEXTURE_PATH, cameraMesh)) {
        printf("Warning: Failed to load security camera model\n");
        success = false;
    }
    
    if (!loadOBJMesh(CONSOLE_MODEL_PATH, CONSOLE_TEXTURE_PATH, consoleMesh)) {
        printf("Warning: Failed to load console model\n");
        success = false;
    }
    
    // Load robot guard OBJ model for Neo Tokyo
    if (!loadOBJMesh(GUARD_MODEL_PATH, GUARD_TEXTURE_PATH, guardMesh)) {
        printf("Warning: Failed to load guard model\n");
        // Fallback: load just the texture for placeholder geometry
        guardTexture = loadTextureFromFile(GUARD_TEXTURE_PATH);
        if (guardTexture.valid()) {
            guardTextureLoaded = true;
        }
    } else {
        guardTextureLoaded = true;
        printf("Loaded guard model: %s\n", GUARD_MODEL_PATH);
    }
    
    // Load Anubis enemy OBJ model for Ancient Egypt
    if (!loadOBJMesh(ANUBIS_MODEL_PATH, ANUBIS_TEXTURE_PATH, anubisMesh)) {
        printf("Warning: Failed to load Anubis enemy model\n");
    } else {
        printf("Loaded Anubis model: %s\n", ANUBIS_MODEL_PATH);
    }
    
    // Load Japanese neon street sign model for Neo-Tokyo
    if (!loadOBJMesh(NEON_SIGN_MODEL_PATH, NEON_SIGN_TEXTURE_PATH, neonSignMesh)) {
        printf("Warning: Failed to load neon sign model\n");
    } else {
        printf("Loaded neon sign model: %s\n", NEON_SIGN_MODEL_PATH);
    }
    
    // Load Egyptian coffin/tomb model
    if (!loadOBJMesh(TOMB_MODEL_PATH, TOMB_TEXTURE_PATH, tombMesh)) {
        printf("Warning: Failed to load tomb/coffin model\n");
    } else {
        printf("Loaded tomb model: %s\n", TOMB_MODEL_PATH);
    }
    
    // Load Egyptian column/pillar model
    if (!loadOBJMesh(COLUMN_MODEL_PATH, COLUMN_TEXTURE_PATH, columnMesh)) {
        printf("Warning: Failed to load column model\n");
    } else {
        printf("Loaded column model: %s\n", COLUMN_MODEL_PATH);
    }
    
    return success;
}

void drawLoadedCharacter() {
    if (!mainCharacterMesh.loaded || mainCharacterMesh.positions.empty() || mainCharacterMesh.indices.empty()) {
        return;
    }

    bool hasUV = !mainCharacterMesh.texcoords.empty() && mainCharacterMesh.texcoords.size() / 2 >= mainCharacterMesh.positions.size() / 3;
    bool hasNormals = !mainCharacterMesh.normals.empty();

    if (mainCharacterMesh.albedo.valid()) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, mainCharacterMesh.albedo.id);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < mainCharacterMesh.indices.size(); ++i) {
        unsigned int idx = mainCharacterMesh.indices[i];
        if (hasUV) {
            float u = mainCharacterMesh.texcoords[idx * 2];
            float v = mainCharacterMesh.texcoords[idx * 2 + 1];
            glTexCoord2f(u, v);
        }
        if (hasNormals) {
            float nx = mainCharacterMesh.normals[idx * 3];
            float ny = mainCharacterMesh.normals[idx * 3 + 1];
            float nz = mainCharacterMesh.normals[idx * 3 + 2];
            glNormal3f(nx, ny, nz);
        }
        float vx = mainCharacterMesh.positions[idx * 3];
        float vy = mainCharacterMesh.positions[idx * 3 + 1];
        float vz = mainCharacterMesh.positions[idx * 3 + 2];
        glVertex3f(vx, vy, vz);
    }
    glEnd();

    if (mainCharacterMesh.albedo.valid()) {
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }
}

void useTexture(const TextureResource& tex) {
    if (tex.valid()) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, tex.id);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D);
    }
}

// =============================================================================
// Utility Functions
// =============================================================================
void playSFX(const char* soundName) {
    // Placeholder for SFX - would integrate with audio library
    // printf("SFX: %s\n", soundName);
}

// =============================================================================
// Rendering Functions
// =============================================================================
// Forward declarations
void drawTorus(float outerRadius, float innerRadius, int sides, int rings);

void drawCube(float size) {
    float s = size * 0.5f;
    glBegin(GL_QUADS);
    // Front
    glNormal3f(0, 0, 1);
    glVertex3f(-s, -s, s); glVertex3f(s, -s, s); glVertex3f(s, s, s); glVertex3f(-s, s, s);
    // Back
    glNormal3f(0, 0, -1);
    glVertex3f(-s, -s, -s); glVertex3f(-s, s, -s); glVertex3f(s, s, -s); glVertex3f(s, -s, -s);
    // Top
    glNormal3f(0, 1, 0);
    glVertex3f(-s, s, -s); glVertex3f(-s, s, s); glVertex3f(s, s, s); glVertex3f(s, s, -s);
    // Bottom
    glNormal3f(0, -1, 0);
    glVertex3f(-s, -s, -s); glVertex3f(s, -s, -s); glVertex3f(s, -s, s); glVertex3f(-s, -s, s);
    // Right
    glNormal3f(1, 0, 0);
    glVertex3f(s, -s, -s); glVertex3f(s, s, -s); glVertex3f(s, s, s); glVertex3f(s, -s, s);
    // Left
    glNormal3f(-1, 0, 0);
    glVertex3f(-s, -s, -s); glVertex3f(-s, -s, s); glVertex3f(-s, s, s); glVertex3f(-s, s, -s);
    glEnd();
}

void drawPyramid(float baseSize, float height) {
    float s = baseSize * 0.5f;
    float h = height;
    float texScale = 2.0f;  // Texture repeat scale for pyramid faces
    
    glBegin(GL_TRIANGLES);
    // Base (square base made of 2 triangles)
    glNormal3f(0, -1, 0);
    glTexCoord2f(0, 0); glVertex3f(-s, 0, -s);
    glTexCoord2f(texScale, 0); glVertex3f(s, 0, -s);
    glTexCoord2f(texScale, texScale); glVertex3f(s, 0, s);
    
    glTexCoord2f(0, 0); glVertex3f(-s, 0, -s);
    glTexCoord2f(texScale, texScale); glVertex3f(s, 0, s);
    glTexCoord2f(0, texScale); glVertex3f(-s, 0, s);
    
    // Front face
    Vector3f v1(-s, 0, s);
    Vector3f v2(s, 0, s);
    Vector3f v3(0, h, 0);
    Vector3f normal = (v2 - v1).cross(v3 - v1).unit();
    glNormal3f(normal.x, normal.y, normal.z);
    glTexCoord2f(0, 0); glVertex3f(-s, 0, s);
    glTexCoord2f(texScale, 0); glVertex3f(s, 0, s);
    glTexCoord2f(texScale * 0.5f, texScale); glVertex3f(0, h, 0);
    
    // Back face
    v1 = Vector3f(s, 0, -s);
    v2 = Vector3f(-s, 0, -s);
    v3 = Vector3f(0, h, 0);
    normal = (v2 - v1).cross(v3 - v1).unit();
    glNormal3f(normal.x, normal.y, normal.z);
    glTexCoord2f(0, 0); glVertex3f(s, 0, -s);
    glTexCoord2f(texScale, 0); glVertex3f(-s, 0, -s);
    glTexCoord2f(texScale * 0.5f, texScale); glVertex3f(0, h, 0);
    
    // Right face
    v1 = Vector3f(s, 0, s);
    v2 = Vector3f(s, 0, -s);
    v3 = Vector3f(0, h, 0);
    normal = (v2 - v1).cross(v3 - v1).unit();
    glNormal3f(normal.x, normal.y, normal.z);
    glTexCoord2f(0, 0); glVertex3f(s, 0, s);
    glTexCoord2f(texScale, 0); glVertex3f(s, 0, -s);
    glTexCoord2f(texScale * 0.5f, texScale); glVertex3f(0, h, 0);
    
    // Left face
    v1 = Vector3f(-s, 0, -s);
    v2 = Vector3f(-s, 0, s);
    v3 = Vector3f(0, h, 0);
    normal = (v2 - v1).cross(v3 - v1).unit();
    glNormal3f(normal.x, normal.y, normal.z);
    glTexCoord2f(0, 0); glVertex3f(-s, 0, -s);
    glTexCoord2f(texScale, 0); glVertex3f(-s, 0, s);
    glTexCoord2f(texScale * 0.5f, texScale); glVertex3f(0, h, 0);
    
    glEnd();
}

void drawSphere(float radius, int slices, int stacks) {
    GLUquadric* quad = gluNewQuadric();
    gluSphere(quad, radius, slices, stacks);
    gluDeleteQuadric(quad);
}

void drawCylinder(float baseRadius, float topRadius, float height, int slices, int stacks) {
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, baseRadius, topRadius, height, slices, stacks);
    gluDeleteQuadric(quad);
}

void drawTemporalCrystal(Vector3f pos, float rotation, float bob) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y + bob, pos.z);
    glRotatef(rotation, 0, 1, 0);

    // Emissive material for crystal
    float emissive[4] = {0.3f, 0.6f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
    glColor3f(0.2f, 0.5f, 1.0f);

    // Crystal shape (octahedron-like)
    glBegin(GL_TRIANGLES);
    // Top pyramid
    glNormal3f(0, 1, 0);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0.3f, 0, 0);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(0, 0, -0.3f);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(0.3f, 0, 0);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0, 0, -0.3f);
    glVertex3f(0, 0.4f, 0);
    glVertex3f(0, 0, -0.3f);
    glVertex3f(0.3f, 0, 0);
    // Bottom pyramid
    glNormal3f(0, -1, 0);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(0.3f, 0, 0);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(0, 0, -0.3f);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(0.3f, 0, 0);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0, 0, 0.3f);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(0, 0, -0.3f);
    glVertex3f(-0.3f, 0, 0);
    glVertex3f(0, -0.4f, 0);
    glVertex3f(0.3f, 0, 0);
    glVertex3f(0, 0, -0.3f);
    glEnd();

    float noEmissive[4] = {0, 0, 0, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    glPopMatrix();
}

void drawGoldenScarab(Vector3f pos, float rotation, float bob) {
    if (scarabMesh.loaded) {
        // Use loaded OBJ model
        glPushMatrix();
        glTranslatef(pos.x, pos.y + bob, pos.z);
        glRotatef(rotation, 0, 1, 0);
        
        // Golden emissive glow
        float emissive[4] = {0.5f, 0.4f, 0.1f, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
        
        // Set golden color (will show if texture fails)
        glColor3f(0.9f, 0.75f, 0.2f);
        
        // Calculate scale to make scarab about 0.5 units tall
        float targetSize = 0.5f;
        float modelHeight = scarabMesh.boundsMax.y - scarabMesh.boundsMin.y;
        float scarabScale = targetSize / std::max(0.001f, modelHeight);
        glScalef(scarabScale, scarabScale, scarabScale);
        
        // Center the mesh
        glTranslatef(-scarabMesh.boundsCenter.x, -scarabMesh.boundsMin.y, -scarabMesh.boundsCenter.z);
        
        // Draw the mesh
        bool hasUV = !scarabMesh.texcoords.empty() && scarabMesh.texcoords.size() / 2 >= scarabMesh.positions.size() / 3;
        bool hasNormals = !scarabMesh.normals.empty();
        
        if (scarabMesh.albedo.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, scarabMesh.albedo.id);
        }
        
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < scarabMesh.indices.size(); i++) {
            unsigned int idx = scarabMesh.indices[i];
            if (hasUV && idx * 2 + 1 < scarabMesh.texcoords.size()) {
                glTexCoord2f(scarabMesh.texcoords[idx * 2], scarabMesh.texcoords[idx * 2 + 1]);
            }
            if (hasNormals && idx * 3 + 2 < scarabMesh.normals.size()) {
                glNormal3f(scarabMesh.normals[idx * 3], scarabMesh.normals[idx * 3 + 1], scarabMesh.normals[idx * 3 + 2]);
            }
            if (idx * 3 + 2 < scarabMesh.positions.size()) {
                glVertex3f(scarabMesh.positions[idx * 3], scarabMesh.positions[idx * 3 + 1], scarabMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        glDisable(GL_TEXTURE_2D);
        float noEmissive[4] = {0, 0, 0, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
        glPopMatrix();
    } else {
        // Fallback: Original placeholder
        glPushMatrix();
        glTranslatef(pos.x, pos.y + bob, pos.z);
        glRotatef(rotation, 0, 1, 0);

        float emissive[4] = {0.8f, 0.6f, 0.2f, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
        glColor3f(0.9f, 0.7f, 0.1f);

        glScalef(0.4f, 0.2f, 0.6f);
        drawSphere(1.0f, 16, 16);

        glPushMatrix();
        glTranslatef(0, 0, 0.7f);
        glScalef(0.3f, 0.3f, 0.3f);
        drawSphere(1.0f, 12, 12);
        glPopMatrix();

        glPushMatrix();
        glTranslatef(0, 0.15f, 0);
        glScalef(0.8f, 0.1f, 1.2f);
        glColor3f(0.95f, 0.75f, 0.15f);
        drawSphere(1.0f, 16, 16);
        glPopMatrix();

        float noEmissive[4] = {0, 0, 0, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
        glPopMatrix();
    }
}

void drawPlayer() {
    // Only draw player in third-person mode
    if (camera.mode == CAMERA_FIRST_PERSON) {
        return;
    }
    
    glPushMatrix();
    // Translate to player position (player.position.y is at player's center)
    // Move down by half player height to place at feet level
    glTranslatef(player.position.x, player.position.y - PLAYER_HEIGHT * 0.5f, player.position.z);
    
    // Rotate player to face the direction they're moving (yaw angle)
    // player.yaw is smoothly updated in Player::update() to face movement direction
    glRotatef(player.yaw + 180.0f, 0, 1, 0);  // Rotate around Y-axis (+180 to face forward)

    if (mainCharacterMesh.loaded) {
        // For OBJ model: Apply proper transformations
        
        // Scale the model to fit player height (increased scale)
        float modelScale = mainCharacterMesh.scale * 1.5f;  // Larger scale
        glScalef(modelScale, modelScale, modelScale);
        
        // OBJ models are typically Y-up, which matches our coordinate system
        // Just rotate 180 degrees if the model faces the wrong direction
        glRotatef(180.0f, 0, 1, 0);
        
        // Center the model horizontally and place feet at ground level
        glTranslatef(-mainCharacterMesh.boundsCenter.x,
                     -mainCharacterMesh.boundsMin.y,   // Place feet at origin (ground level)
                     -mainCharacterMesh.boundsCenter.z);
        
        // Draw the character model
        drawLoadedCharacter();
    } else {
        // Fallback: Simple humanoid character with higher poly count
        
        // Body color
        glColor3f(0.2f, 0.4f, 0.8f);
        
        // Torso (cylinder-like, higher poly)
        glPushMatrix();
        glTranslatef(0, 0.9f, 0);
        glScalef(0.35f, 0.5f, 0.2f);
        glutSolidSphere(1.0f, 16, 16);  // Higher poly sphere for torso
        glPopMatrix();
        
        // Head
        glPushMatrix();
        glTranslatef(0, 1.6f, 0);
        glColor3f(0.9f, 0.75f, 0.6f);  // Skin color
        glutSolidSphere(0.2f, 16, 16);  // Higher poly head
        glPopMatrix();
        
        // Neck
        glPushMatrix();
        glTranslatef(0, 1.35f, 0);
        glColor3f(0.9f, 0.75f, 0.6f);
        glScalef(0.08f, 0.1f, 0.08f);
        glutSolidSphere(1.0f, 12, 12);
        glPopMatrix();
        
        // Left arm
        glPushMatrix();
        glTranslatef(-0.45f, 0.95f, 0);
        glColor3f(0.2f, 0.4f, 0.8f);
        glScalef(0.1f, 0.35f, 0.1f);
        glutSolidSphere(1.0f, 12, 12);
        glPopMatrix();
        
        // Right arm
        glPushMatrix();
        glTranslatef(0.45f, 0.95f, 0);
        glScalef(0.1f, 0.35f, 0.1f);
        glutSolidSphere(1.0f, 12, 12);
        glPopMatrix();
        
        // Left leg
        glPushMatrix();
        glTranslatef(-0.15f, 0.35f, 0);
        glColor3f(0.15f, 0.15f, 0.3f);  // Darker pants color
        glScalef(0.12f, 0.4f, 0.12f);
        glutSolidSphere(1.0f, 12, 12);
        glPopMatrix();
        
        // Right leg
        glPushMatrix();
        glTranslatef(0.15f, 0.35f, 0);
        glScalef(0.12f, 0.4f, 0.12f);
        glutSolidSphere(1.0f, 12, 12);
        glPopMatrix();
        
        // Face direction indicator (eyes looking forward)
        glPushMatrix();
        glColor3f(1.0f, 1.0f, 1.0f);  // White eyes
        glTranslatef(-0.07f, 1.63f, 0.15f);
        glutSolidSphere(0.04f, 8, 8);
        glTranslatef(0.14f, 0, 0);
        glutSolidSphere(0.04f, 8, 8);
        // Pupils
        glColor3f(0.1f, 0.1f, 0.1f);
        glTranslatef(-0.14f, 0, 0.03f);
        glutSolidSphere(0.02f, 6, 6);
        glTranslatef(0.14f, 0, 0);
        glutSolidSphere(0.02f, 6, 6);
        glPopMatrix();
    }

    glPopMatrix();
}

void drawLaserGrid(Vector3f pos, float size, bool active) {
    if (!active) return;

    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(2.0f);
    
    // Vertical lines
    for (int i = 0; i <= 5; i++) {
        float x = (i / 5.0f - 0.5f) * size;
        glBegin(GL_LINES);
        glVertex3f(x, 0, -size * 0.5f);
        glVertex3f(x, size, -size * 0.5f);
        glEnd();
        glBegin(GL_LINES);
        glVertex3f(x, 0, size * 0.5f);
        glVertex3f(x, size, size * 0.5f);
        glEnd();
    }
    
    // Horizontal lines
    for (int i = 0; i <= 5; i++) {
        float y = (i / 5.0f) * size;
        glBegin(GL_LINES);
        glVertex3f(-size * 0.5f, y, -size * 0.5f);
        glVertex3f(size * 0.5f, y, -size * 0.5f);
        glEnd();
        glBegin(GL_LINES);
        glVertex3f(-size * 0.5f, y, size * 0.5f);
        glVertex3f(size * 0.5f, y, size * 0.5f);
        glEnd();
    }
    
    glLineWidth(1.0f);
    glPopMatrix();
}

void drawSecurityCamera(Vector3f pos, float rotation, float sweepAngle) {
    if (cameraMesh.loaded) {
        // Use loaded OBJ model
        // Calculate scale to make camera about 0.8 units tall
        float targetSize = 0.8f;
        float modelHeight = cameraMesh.boundsMax.y - cameraMesh.boundsMin.y;
        float cameraScale = targetSize / std::max(0.001f, modelHeight);
        
        drawOBJMesh(cameraMesh, pos, rotation, cameraScale);
        
        // ALWAYS draw detection cone/laser (red when alarm, green when normal)
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glRotatef(rotation, 0, 1, 0);
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        if (alarmActive) {
            // Red cone when alarm is active (wider sweep)
            glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
        } else {
            // Green cone when normal (narrower sweep)
            glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
        }
        
        // Detection cone pointing in +X direction (forward for this security camera model)
        glBegin(GL_TRIANGLES);
        glVertex3f(0, 0, 0);
        float angle1 = -sweepAngle * 0.5f;
        float angle2 = sweepAngle * 0.5f;
        float dist = 8.0f;
        // Cone points in +X direction (matches camera lens direction)
        glVertex3f(cos(DEG2RAD(angle1)) * dist, -2.0f, sin(DEG2RAD(angle1)) * dist);
        glVertex3f(cos(DEG2RAD(angle2)) * dist, -2.0f, sin(DEG2RAD(angle2)) * dist);
        glEnd();
        
        glDisable(GL_BLEND);
        glPopMatrix();
    } else {
        // Fallback: Original placeholder
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glRotatef(rotation, 0, 1, 0);

        glColor3f(0.3f, 0.3f, 0.3f);
        
        glPushMatrix();
        glTranslatef(0, 0, 0);
        drawCube(0.4f);
        glPopMatrix();

        glPushMatrix();
        glTranslatef(0, 0, 0.25f);
        glColor3f(0.1f, 0.1f, 0.1f);
        drawSphere(0.15f, 12, 12);
        glPopMatrix();

        // ALWAYS draw detection cone/laser (red when alarm, green when normal)
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        if (alarmActive) {
            glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
        } else {
            glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
        }
        
        glBegin(GL_TRIANGLES);
        glVertex3f(0, 0, 0);
        float angle1 = -sweepAngle * 0.5f;
        float angle2 = sweepAngle * 0.5f;
        float dist = 8.0f;
        glVertex3f(sin(DEG2RAD(angle1)) * dist, -2.0f, cos(DEG2RAD(angle1)) * dist);
        glVertex3f(sin(DEG2RAD(angle2)) * dist, -2.0f, cos(DEG2RAD(angle2)) * dist);
        glEnd();
        
        glDisable(GL_BLEND);

        glPopMatrix();
    }
}

void drawControlConsole(Vector3f pos, bool activated) {
    if (consoleMesh.loaded) {
        // Use loaded OBJ model
        // Calculate scale to make console about 1.5 units tall
        float targetSize = 1.5f;
        float modelHeight = consoleMesh.boundsMax.y - consoleMesh.boundsMin.y;
        float consoleScale = targetSize / std::max(0.001f, modelHeight);
        
        // Add glow if activated with animation effect
        if (activated) {
            float emissiveIntensity = 0.3f;
            // Enhanced glow during activation animation
            if (consoleActivationAnimActive) {
                float animTime = gameTime - consoleActivationTime;
                emissiveIntensity = 0.3f + 0.5f * sin(animTime * 10.0f) * (1.0f - animTime / 1.5f);
            }
            float emissive[4] = {0.0f, emissiveIntensity, emissiveIntensity * 0.3f, 1.0f};
            glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
        }
        
        drawOBJMesh(consoleMesh, pos, 0.0f, consoleScale);
        
        if (activated) {
            float noEmissive[4] = {0, 0, 0, 1.0f};
            glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
        }
        
        // Draw activation particle effect
        if (consoleActivationAnimActive) {
            float animTime = gameTime - consoleActivationTime;
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDisable(GL_LIGHTING);
            
            // Rising particles
            for (int i = 0; i < 8; i++) {
                float angle = (i / 8.0f) * 2.0f * M_PI + animTime * 3.0f;
                float radius = 1.0f + sin(animTime * 5.0f + i) * 0.3f;
                float height = animTime * 4.0f + sin(i * 1.5f) * 0.5f;
                float alpha = 1.0f - animTime / 1.5f;
                
                glPushMatrix();
                glTranslatef(pos.x + cos(angle) * radius, pos.y + height, pos.z + sin(angle) * radius);
                glColor4f(0.0f, 1.0f, 0.5f, alpha * 0.7f);
                drawSphere(0.1f, 6, 6);
                glPopMatrix();
            }
            
            glEnable(GL_LIGHTING);
            glDisable(GL_BLEND);
        }
    } else {
        // Fallback: Original placeholder
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);

        glColor3f(0.1f, 0.3f, 0.5f);
        
        drawCube(1.5f);
        
        glPushMatrix();
        glTranslatef(0, 0.8f, 0.76f);
        if (activated) {
            // Flash green during activation
            if (consoleActivationAnimActive) {
                float animTime = gameTime - consoleActivationTime;
                float flash = sin(animTime * 15.0f) * 0.5f + 0.5f;
                glColor3f(flash * 0.5f, 1.0f, flash * 0.5f);
            } else {
                glColor3f(0.0f, 1.0f, 0.0f);
            }
        } else {
            glColor3f(0.2f, 0.2f, 0.4f);
        }
        drawCube(1.0f);
        glPopMatrix();

        glPopMatrix();
    }
}

// Draw Security Robot - Futuristic robot guard for Neo Tokyo
void drawSecurityRobot(const SecurityRobot& robot) {
    glPushMatrix();
    glTranslatef(robot.position.x, robot.position.y, robot.position.z);
    glRotatef(robot.yaw, 0, 1, 0);
    
    // Use guard OBJ model if loaded
    if (guardMesh.loaded) {
        // Calculate scale to make guard about 2.0 units tall
        float targetSize = 2.0f;
        float modelHeight = guardMesh.boundsMax.y - guardMesh.boundsMin.y;
        float guardScale = targetSize / std::max(0.001f, modelHeight);
        
        // Center and draw the model
        glPushMatrix();
        glScalef(guardScale, guardScale, guardScale);
        glTranslatef(-guardMesh.boundsCenter.x, -guardMesh.boundsMin.y, -guardMesh.boundsCenter.z);
        
        bool hasUV = !guardMesh.texcoords.empty();
        bool hasNormals = !guardMesh.normals.empty();
        
        if (guardMesh.albedo.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, guardMesh.albedo.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        } else {
            glDisable(GL_TEXTURE_2D);
            glColor3f(0.3f, 0.3f, 0.35f);
        }
        
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < guardMesh.indices.size(); i++) {
            unsigned int idx = guardMesh.indices[i];
            if (hasUV && idx * 2 + 1 < guardMesh.texcoords.size()) {
                glTexCoord2f(guardMesh.texcoords[idx * 2], guardMesh.texcoords[idx * 2 + 1]);
            }
            if (hasNormals && idx * 3 + 2 < guardMesh.normals.size()) {
                glNormal3f(guardMesh.normals[idx * 3], guardMesh.normals[idx * 3 + 1], guardMesh.normals[idx * 3 + 2]);
            }
            if (idx * 3 + 2 < guardMesh.positions.size()) {
                glVertex3f(guardMesh.positions[idx * 3], guardMesh.positions[idx * 3 + 1], guardMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    } else {
        // Fallback: Use placeholder geometry with texture
        bool useTexture = guardTextureLoaded && guardTexture.valid();
        if (useTexture) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, guardTexture.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        
        // Robot body - metallic/armored appearance
        float specular[4] = {0.8f, 0.8f, 0.9f, 1.0f};
        glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
        glMaterialf(GL_FRONT, GL_SHININESS, 80.0f);
        
        if (!useTexture) glColor3f(0.25f, 0.25f, 0.3f);
        
        // Main body (torso)
        glPushMatrix();
        glScalef(0.8f, 1.2f, 0.5f);
        drawCube(1.0f);
        glPopMatrix();
        
        // Head
        glPushMatrix();
        glTranslatef(0, 0.9f, 0);
        if (!useTexture) glColor3f(0.3f, 0.3f, 0.35f);
        drawSphere(0.35f, 16, 16);
        glPopMatrix();
        
        // Arms
        if (!useTexture) glColor3f(0.22f, 0.22f, 0.27f);
        glPushMatrix();
        glTranslatef(-0.7f, 0.0f, 0);
        glScalef(0.15f, 0.6f, 0.15f);
        drawCube(1.0f);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.7f, 0.0f, 0);
        glScalef(0.15f, 0.6f, 0.15f);
        drawCube(1.0f);
        glPopMatrix();
        
        // Legs
        if (!useTexture) glColor3f(0.2f, 0.2f, 0.25f);
        glPushMatrix();
        glTranslatef(-0.25f, -0.9f, 0);
        glScalef(0.2f, 0.6f, 0.2f);
        drawCube(1.0f);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.25f, -0.9f, 0);
        glScalef(0.2f, 0.6f, 0.2f);
        drawCube(1.0f);
        glPopMatrix();
        
        glDisable(GL_TEXTURE_2D);
    }
    
    // Warning lights on shoulders (always on; brighter when alarm/chasing)
    float noEmissive[4] = {0, 0, 0, 1.0f};
    float blink = (sin(gameTime * 10.0f) > 0) ? 1.0f : 0.5f;
    if (!alarmActive && !robot.chasing) blink = 0.5f;
    float warningEmissive[4] = {1.0f * blink, 0.3f * blink, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, warningEmissive);
    glColor3f(1.0f * blink, 0.3f * blink, 0.0f);
    
    glPushMatrix();
    glTranslatef(-0.6f, 0.55f, 0);
    drawSphere(0.08f, 8, 8);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.6f, 0.55f, 0);
    drawSphere(0.08f, 8, 8);
    glPopMatrix();
    
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    
    // Reset material
    float defaultSpec[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glMaterialfv(GL_FRONT, GL_SPECULAR, defaultSpec);
    glMaterialf(GL_FRONT, GL_SHININESS, 32.0f);
    
    glPopMatrix();
    
    // FoV cone on ground (red transparent)
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1.0f, 0.0f, 0.0f, 0.25f);
    glPushMatrix();
    glTranslatef(robot.position.x, GROUND_Y + 0.05f, robot.position.z);
    glRotatef(robot.yaw, 0, 1, 0);
    float fov = robot.fovDegrees;
    float range = robot.detectDistance;
    float half = fov * 0.5f;
    float a = half * 3.14159f / 180.0f;
    float b = -half * 3.14159f / 180.0f;
    glBegin(GL_TRIANGLES);
    glNormal3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(sin(a) * range, 0, cos(a) * range);
    glVertex3f(sin(b) * range, 0, cos(b) * range);
    glEnd();
    glPopMatrix();
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
}

void drawExitPortal(Vector3f pos, bool unlocked, float pulse) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);

    // Portal ring - enhanced scale during activation animation
    float scale = 1.0f + pulse * 0.3f;
    
    // Portal activation animation effect
    if (portalActivationAnimActive && unlocked) {
        float animTime = gameTime - portalActivationTime;
        // Rapid pulsing during activation
        scale += sin(animTime * 15.0f) * 0.2f * (1.0f - animTime / 2.0f);
    }
    
    glScalef(scale, scale, scale);

    // Emissive material - enhanced during activation
    float emissiveBoost = 1.0f;
    if (portalActivationAnimActive && unlocked) {
        float animTime = gameTime - portalActivationTime;
        emissiveBoost = 1.0f + sin(animTime * 10.0f) * 0.5f;
    }
    
    float emissive[4] = {
        (0.2f + pulse * 0.3f) * emissiveBoost, 
        (0.4f + pulse * 0.4f) * emissiveBoost, 
        (0.8f + pulse * 0.2f) * emissiveBoost, 
        1.0f
    };
    glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
    
    if (unlocked) {
        glColor3f(0.2f + pulse * 0.3f, 0.6f + pulse * 0.4f, 1.0f);
    } else {
        glColor3f(0.1f, 0.1f, 0.3f);
        float noEmissive[4] = {0, 0, 0, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    }

    // Outer ring
    drawTorus(2.0f, 0.3f, 32, 16);
    
    // Inner portal surface
    glPushMatrix();
    glRotatef(90, 1, 0, 0);
    if (unlocked) {
        glColor4f(0.3f, 0.5f, 1.0f, 0.7f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawSphere(1.5f, 32, 32);
        glDisable(GL_BLEND);
    }
    glPopMatrix();
    
    // Draw activation particle burst effect
    if (portalActivationAnimActive && unlocked) {
        float animTime = gameTime - portalActivationTime;
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Expanding ring of particles
        for (int i = 0; i < 16; i++) {
            float angle = (i / 16.0f) * 2.0f * M_PI;
            float expandRadius = 2.0f + animTime * 3.0f;
            float alpha = 1.0f - animTime / 2.0f;
            
            glPushMatrix();
            glTranslatef(cos(angle) * expandRadius, sin(animTime * 5.0f + i * 0.5f) * 0.5f, sin(angle) * expandRadius);
            glColor4f(0.3f, 0.6f, 1.0f, alpha * 0.8f);
            drawSphere(0.15f - animTime * 0.05f, 6, 6);
            glPopMatrix();
        }
        
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
    }

    float noEmissive[4] = {0, 0, 0, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    glPopMatrix();
}

void drawTorus(float outerRadius, float innerRadius, int sides, int rings) {
    for (int i = 0; i < rings; i++) {
        float theta1 = (float)i / rings * 2.0f * M_PI;
        float theta2 = (float)(i + 1) / rings * 2.0f * M_PI;
        
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= sides; j++) {
            float phi = (float)j / sides * 2.0f * M_PI;
            
            for (int k = 0; k < 2; k++) {
                float theta = (k == 0) ? theta1 : theta2;
                float r = outerRadius + innerRadius * cos(phi);
                float x = r * cos(theta);
                float y = r * sin(theta);
                float z = innerRadius * sin(phi);
                
                Vector3f normal = Vector3f(cos(theta) * cos(phi), sin(theta) * cos(phi), sin(phi));
                normal = normal.unit();
                glNormal3f(normal.x, normal.y, normal.z);
                glVertex3f(x, y, z);
            }
        }
        glEnd();
    }
}

void drawPressurePlate(Vector3f pos, bool triggered) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);

    if (triggered) {
        glColor3f(1.0f, 0.2f, 0.2f);
        glTranslatef(0, -0.1f, 0);
    } else {
        glColor3f(0.4f, 0.3f, 0.2f);
    }

    drawCube(1.0f);
    glPopMatrix();
}

// =============================================================================
// Enhanced Pressure Plate Trap Drawing (Egypt) - With trap door and spikes
// =============================================================================
void drawPressurePlateTrap(const PressurePlateTrap& trap) {
    float halfSize = PressurePlateTrap::PLATE_SIZE * 0.5f;
    
    glPushMatrix();
    glTranslatef(trap.position.x, trap.position.y, trap.position.z);
    
    // Draw the pit hole (dark area beneath)
    if (trap.trapDoorOpen > 0.01f) {
        glColor3f(0.05f, 0.02f, 0.02f);  // Very dark pit
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glEnd();
        
        // Pit walls
        glColor3f(0.2f, 0.15f, 0.1f);
        // Front wall
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glVertex3f(halfSize, 0, halfSize);
        glVertex3f(-halfSize, 0, halfSize);
        glEnd();
        // Back wall
        glBegin(GL_QUADS);
        glNormal3f(0, 0, -1);
        glVertex3f(-halfSize, 0, -halfSize);
        glVertex3f(halfSize, 0, -halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glEnd();
        // Left wall
        glBegin(GL_QUADS);
        glNormal3f(-1, 0, 0);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glVertex3f(-halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glVertex3f(-halfSize, 0, halfSize);
        glVertex3f(-halfSize, 0, -halfSize);
        glEnd();
        // Right wall
        glBegin(GL_QUADS);
        glNormal3f(1, 0, 0);
        glVertex3f(halfSize, 0, -halfSize);
        glVertex3f(halfSize, 0, halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, halfSize);
        glVertex3f(halfSize, -PressurePlateTrap::PIT_DEPTH, -halfSize);
        glEnd();
        
        // Draw spikes at bottom of pit
        if (trap.spikeHeight > 0.01f) {
            glColor3f(0.5f, 0.5f, 0.5f);  // Metallic grey spikes
            float spikeSpacing = 0.5f;
            for (float sx = -halfSize + 0.3f; sx < halfSize; sx += spikeSpacing) {
                for (float sz = -halfSize + 0.3f; sz < halfSize; sz += spikeSpacing) {
                    glPushMatrix();
                    glTranslatef(sx, -PressurePlateTrap::PIT_DEPTH, sz);
                    glRotatef(-90, 1, 0, 0);
                    glutSolidCone(0.1f, trap.spikeHeight, 8, 1);
                    glPopMatrix();
                }
            }
        }
    }
    
    // Draw trap door panels (hinged on opposite sides)
    if (trap.trapDoorOpen < 0.99f) {
        float doorAngle = trap.trapDoorOpen * 90.0f;  // 0 to 90 degrees
        
        // Visual telegraph: plate glows red when about to trigger
        if (!trap.triggered) {
            // Normal plate color with warning pattern
            glColor3f(0.5f, 0.35f, 0.2f);
            
            // Draw warning symbols on plate (hieroglyphic-style marks)
            glPushMatrix();
            glTranslatef(0, 0.01f, 0);
            glColor3f(0.7f, 0.5f, 0.2f);  // Gold warning color
            // Draw X pattern
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            glVertex3f(-halfSize * 0.7f, 0.02f, -halfSize * 0.7f);
            glVertex3f(halfSize * 0.7f, 0.02f, halfSize * 0.7f);
            glVertex3f(-halfSize * 0.7f, 0.02f, halfSize * 0.7f);
            glVertex3f(halfSize * 0.7f, 0.02f, -halfSize * 0.7f);
            glEnd();
            glLineWidth(1.0f);
            glPopMatrix();
        } else {
            glColor3f(0.8f, 0.2f, 0.1f);  // Red when triggered
        }
        
        // Left door panel (hinges on left side)
        glPushMatrix();
        glTranslatef(-halfSize, 0, 0);
        glRotatef(doorAngle, 0, 0, 1);
        glTranslatef(halfSize * 0.5f, -0.1f, 0);
        glScalef(halfSize, 0.2f, PressurePlateTrap::PLATE_SIZE);
        drawCube(1.0f);
        glPopMatrix();
        
        // Right door panel (hinges on right side)
        glPushMatrix();
        glTranslatef(halfSize, 0, 0);
        glRotatef(-doorAngle, 0, 0, 1);
        glTranslatef(-halfSize * 0.5f, -0.1f, 0);
        glScalef(halfSize, 0.2f, PressurePlateTrap::PLATE_SIZE);
        drawCube(1.0f);
        glPopMatrix();
    }
    
    // Draw decorative stone frame around pit
    glColor3f(0.4f, 0.35f, 0.3f);
    float frameWidth = 0.3f;
    // Front frame
    glPushMatrix();
    glTranslatef(0, 0, halfSize + frameWidth * 0.5f);
    glScalef(PressurePlateTrap::PLATE_SIZE + frameWidth * 2, 0.4f, frameWidth);
    drawCube(1.0f);
    glPopMatrix();
    // Back frame
    glPushMatrix();
    glTranslatef(0, 0, -halfSize - frameWidth * 0.5f);
    glScalef(PressurePlateTrap::PLATE_SIZE + frameWidth * 2, 0.4f, frameWidth);
    drawCube(1.0f);
    glPopMatrix();
    // Left frame
    glPushMatrix();
    glTranslatef(-halfSize - frameWidth * 0.5f, 0, 0);
    glScalef(frameWidth, 0.4f, PressurePlateTrap::PLATE_SIZE);
    drawCube(1.0f);
    glPopMatrix();
    // Right frame
    glPushMatrix();
    glTranslatef(halfSize + frameWidth * 0.5f, 0, 0);
    glScalef(frameWidth, 0.4f, PressurePlateTrap::PLATE_SIZE);
    drawCube(1.0f);
    glPopMatrix();
    
    glPopMatrix();
}

// =============================================================================
// Moving Platform with Fatal Pit Drawing (Egypt)
// =============================================================================
void drawMovingPlatformWithPit(const MovingPlatform& platform) {
    // Draw the pit (deadly hole in the ground)
    float pitHalfWidth = MovingPlatform::PIT_WIDTH * 0.5f;
    float pitHalfLength = MovingPlatform::PIT_LENGTH * 0.5f;
    
    glPushMatrix();
    glTranslatef(platform.basePosition.x, platform.basePosition.y, platform.basePosition.z);
    
    // Pit floor (very dark)
    glColor3f(0.02f, 0.01f, 0.01f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glEnd();
    
    // Pit walls with temple texture
    useTexture(templeSurface);
    glColor3f(0.3f, 0.25f, 0.2f);
    // Front wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glTexCoord2f(2, 0); glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glTexCoord2f(2, 3); glVertex3f(pitHalfWidth, 0, pitHalfLength);
    glTexCoord2f(0, 3); glVertex3f(-pitHalfWidth, 0, pitHalfLength);
    glEnd();
    // Back wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 3); glVertex3f(-pitHalfWidth, 0, -pitHalfLength);
    glTexCoord2f(2, 3); glVertex3f(pitHalfWidth, 0, -pitHalfLength);
    glTexCoord2f(2, 0); glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glTexCoord2f(0, 0); glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glEnd();
    // Left wall
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glTexCoord2f(2, 0); glVertex3f(-pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glTexCoord2f(2, 3); glVertex3f(-pitHalfWidth, 0, pitHalfLength);
    glTexCoord2f(0, 3); glVertex3f(-pitHalfWidth, 0, -pitHalfLength);
    glEnd();
    // Right wall
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glTexCoord2f(0, 3); glVertex3f(pitHalfWidth, 0, -pitHalfLength);
    glTexCoord2f(2, 3); glVertex3f(pitHalfWidth, 0, pitHalfLength);
    glTexCoord2f(2, 0); glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, pitHalfLength);
    glTexCoord2f(0, 0); glVertex3f(pitHalfWidth, -MovingPlatform::PIT_DEPTH, -pitHalfLength);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
    
    glPopMatrix();
    
    // Draw the moving platform
    Vector3f platPos = platform.getPlatformPosition();
    float platHalf = MovingPlatform::PLATFORM_SIZE * 0.5f;
    
    glPushMatrix();
    glTranslatef(platPos.x, platPos.y, platPos.z);
    
    // Platform surface with sandstone texture
    useTexture(templeSurface);
    glColor3f(0.6f, 0.5f, 0.4f);
    glBegin(GL_QUADS);
    // Top
    glNormal3f(0, 1, 0);
    glTexCoord2f(0, 0); glVertex3f(-platHalf, 0.3f, -platHalf);
    glTexCoord2f(1, 0); glVertex3f(platHalf, 0.3f, -platHalf);
    glTexCoord2f(1, 1); glVertex3f(platHalf, 0.3f, platHalf);
    glTexCoord2f(0, 1); glVertex3f(-platHalf, 0.3f, platHalf);
    // Bottom
    glNormal3f(0, -1, 0);
    glVertex3f(-platHalf, -0.3f, -platHalf);
    glVertex3f(-platHalf, -0.3f, platHalf);
    glVertex3f(platHalf, -0.3f, platHalf);
    glVertex3f(platHalf, -0.3f, -platHalf);
    // Sides
    glNormal3f(0, 0, 1);
    glVertex3f(-platHalf, -0.3f, platHalf);
    glVertex3f(-platHalf, 0.3f, platHalf);
    glVertex3f(platHalf, 0.3f, platHalf);
    glVertex3f(platHalf, -0.3f, platHalf);
    glNormal3f(0, 0, -1);
    glVertex3f(-platHalf, 0.3f, -platHalf);
    glVertex3f(-platHalf, -0.3f, -platHalf);
    glVertex3f(platHalf, -0.3f, -platHalf);
    glVertex3f(platHalf, 0.3f, -platHalf);
    glNormal3f(-1, 0, 0);
    glVertex3f(-platHalf, -0.3f, -platHalf);
    glVertex3f(-platHalf, 0.3f, -platHalf);
    glVertex3f(-platHalf, 0.3f, platHalf);
    glVertex3f(-platHalf, -0.3f, platHalf);
    glNormal3f(1, 0, 0);
    glVertex3f(platHalf, -0.3f, platHalf);
    glVertex3f(platHalf, 0.3f, platHalf);
    glVertex3f(platHalf, 0.3f, -platHalf);
    glVertex3f(platHalf, -0.3f, -platHalf);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
    
    // Decorative hieroglyphic pattern on top
    glColor3f(0.4f, 0.35f, 0.25f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-platHalf * 0.7f, 0.31f, -platHalf * 0.7f);
    glVertex3f(platHalf * 0.7f, 0.31f, -platHalf * 0.7f);
    glVertex3f(platHalf * 0.7f, 0.31f, platHalf * 0.7f);
    glVertex3f(-platHalf * 0.7f, 0.31f, platHalf * 0.7f);
    glEnd();
    glLineWidth(1.0f);
    
    glPopMatrix();
}

// =============================================================================
// Neon Sign Drawing (Neo-Tokyo)
// =============================================================================
void drawNeonSign(Vector3f pos, float rotY, float scale) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotY, 0, 1, 0);
    
    // Use the loaded neon sign OBJ model
    if (neonSignMesh.loaded && !neonSignMesh.positions.empty()) {
        // Calculate scale to fit desired size
        float modelHeight = neonSignMesh.boundsMax.y - neonSignMesh.boundsMin.y;
        float modelScale = scale / std::max(0.001f, modelHeight);
        
        glPushMatrix();
        glScalef(modelScale, modelScale, modelScale);
        glTranslatef(-neonSignMesh.boundsCenter.x, -neonSignMesh.boundsMin.y, -neonSignMesh.boundsCenter.z);
        
        // Make it glow with emissive material
        float pulse = sin(gameTime * 3.0f) * 0.15f + 0.85f;
        float emissive[4] = {0.9f * pulse, 0.3f * pulse, 0.7f * pulse, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
        
        bool hasUV = !neonSignMesh.texcoords.empty();
        bool hasNormals = !neonSignMesh.normals.empty();
        
        if (neonSignMesh.albedo.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, neonSignMesh.albedo.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        } else {
            glDisable(GL_TEXTURE_2D);
            glColor3f(1.0f, 0.3f, 0.8f);  // Pink neon fallback
        }
        
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < neonSignMesh.indices.size(); i++) {
            unsigned int idx = neonSignMesh.indices[i];
            if (hasUV && idx * 2 + 1 < neonSignMesh.texcoords.size()) {
                glTexCoord2f(neonSignMesh.texcoords[idx * 2], neonSignMesh.texcoords[idx * 2 + 1]);
            }
            if (hasNormals && idx * 3 + 2 < neonSignMesh.normals.size()) {
                glNormal3f(neonSignMesh.normals[idx * 3], neonSignMesh.normals[idx * 3 + 1], neonSignMesh.normals[idx * 3 + 2]);
            }
            if (idx * 3 + 2 < neonSignMesh.positions.size()) {
                glVertex3f(neonSignMesh.positions[idx * 3], neonSignMesh.positions[idx * 3 + 1], neonSignMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        float noEmissive[4] = {0, 0, 0, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    } else {
        // Fallback: Draw a simple glowing rectangle
        float hw = scale * 0.5f;
        float hh = scale * 0.3f;
        
        glColor3f(0.1f, 0.1f, 0.15f);
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glVertex3f(-hw - 0.1f, -hh - 0.1f, -0.1f);
        glVertex3f(hw + 0.1f, -hh - 0.1f, -0.1f);
        glVertex3f(hw + 0.1f, hh + 0.1f, -0.1f);
        glVertex3f(-hw - 0.1f, hh + 0.1f, -0.1f);
        glEnd();
        
        float pulse = sin(gameTime * 3.0f) * 0.1f + 0.9f;
        float emissive[4] = {0.8f * pulse, 0.2f * pulse, 0.6f * pulse, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
        glColor3f(1.0f, 0.3f, 0.8f);
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glVertex3f(-hw, -hh, 0);
        glVertex3f(hw, -hh, 0);
        glVertex3f(hw, hh, 0);
        glVertex3f(-hw, hh, 0);
        glEnd();
        
        float noEmissive[4] = {0, 0, 0, 1.0f};
        glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    }
    
    glPopMatrix();
}

// =============================================================================
// Hieroglyphic Wall Panel Drawing (Egypt)
// =============================================================================
void drawHieroglyphicWall(Vector3f pos, float rotY, float width, float height) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotY, 0, 1, 0);
    
    float hw = width * 0.5f;
    float hh = height * 0.5f;
    
    // Stone wall base
    useTexture(templeSurface);
    glColor3f(0.6f, 0.55f, 0.5f);
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-hw - 0.1f, 0, -0.2f);
    glTexCoord2f(width * 0.3f, 0); glVertex3f(hw + 0.1f, 0, -0.2f);
    glTexCoord2f(width * 0.3f, height * 0.3f); glVertex3f(hw + 0.1f, height, -0.2f);
    glTexCoord2f(0, height * 0.3f); glVertex3f(-hw - 0.1f, height, -0.2f);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    
    // Hieroglyphic panel overlay
    if (hieroglyphicTexture.valid()) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, hieroglyphicTexture.id);
        glColor3f(0.9f, 0.85f, 0.7f);
        
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glTexCoord2f(0, 0); glVertex3f(-hw, 0.2f, 0);
        glTexCoord2f(1, 0); glVertex3f(hw, 0.2f, 0);
        glTexCoord2f(1, 1); glVertex3f(hw, height - 0.2f, 0);
        glTexCoord2f(0, 1); glVertex3f(-hw, height - 0.2f, 0);
        glEnd();
        
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }
    
    // Decorative frame
    glColor3f(0.4f, 0.35f, 0.3f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-hw, 0.1f, 0.01f);
    glVertex3f(hw, 0.1f, 0.01f);
    glVertex3f(hw, height - 0.1f, 0.01f);
    glVertex3f(-hw, height - 0.1f, 0.01f);
    glEnd();
    glLineWidth(1.0f);
    
    glPopMatrix();
}

// =============================================================================
// Tall Egyptian Column Drawing (at corners)
// =============================================================================
void drawTallColumn(Vector3f pos, float height) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    
    // Use column OBJ model if loaded
    if (columnMesh.loaded) {
        // Calculate scale to match desired height
        float modelHeight = columnMesh.boundsMax.y - columnMesh.boundsMin.y;
        float columnScale = height / std::max(0.001f, modelHeight);
        
        glPushMatrix();
        glScalef(columnScale, columnScale, columnScale);
        glTranslatef(-columnMesh.boundsCenter.x, -columnMesh.boundsMin.y, -columnMesh.boundsCenter.z);
        
        // Bind column texture
        if (columnTexture.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, columnTexture.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        } else {
            glColor3f(0.65f, 0.6f, 0.55f); // Stone color fallback
        }
        
        // Render column mesh
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < columnMesh.indices.size(); i++) {
            unsigned int idx = columnMesh.indices[i];
            if (idx * 3 + 2 < columnMesh.normals.size()) {
                glNormal3f(columnMesh.normals[idx * 3], columnMesh.normals[idx * 3 + 1], columnMesh.normals[idx * 3 + 2]);
            }
            if (idx * 2 + 1 < columnMesh.texcoords.size()) {
                glTexCoord2f(columnMesh.texcoords[idx * 2], columnMesh.texcoords[idx * 2 + 1]);
            }
            if (idx * 3 + 2 < columnMesh.positions.size()) {
                glVertex3f(columnMesh.positions[idx * 3], columnMesh.positions[idx * 3 + 1], columnMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    } else {
        // Fallback to procedural column
        useTexture(templeSurface);
        glColor3f(0.65f, 0.6f, 0.55f);
        
        // Column base (wider)
        glPushMatrix();
        glScalef(1.5f, 1.0f, 1.5f);
        drawCylinder(1.0f, 1.0f, 1.0f, 16, 1);
        glPopMatrix();
        
        // Main column shaft
        glPushMatrix();
        glTranslatef(0, 1.0f, 0);
        glRotatef(-90, 1, 0, 0);
        drawCylinder(0.8f, 0.7f, height - 2.0f, 16, 1);
        glPopMatrix();
        
        // Column capital (top, wider)
        glPushMatrix();
        glTranslatef(0, height - 1.0f, 0);
        glRotatef(-90, 1, 0, 0);
        drawCylinder(0.7f, 1.2f, 1.0f, 16, 1);
        glPopMatrix();
        
        // Lotus-style top decoration
        glPushMatrix();
        glTranslatef(0, height, 0);
        glColor3f(0.5f, 0.45f, 0.4f);
        glScalef(1.3f, 0.5f, 1.3f);
        drawSphere(1.0f, 16, 8);
        glPopMatrix();
        
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }
    
    glPopMatrix();
}

// =============================================================================
// Sarcophagus/Tomb Drawing (Egypt)
// =============================================================================
void drawSarcophagus(Vector3f pos, float rotY) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotY, 0, 1, 0);
    
    // Use tomb OBJ model if loaded
    if (tombMesh.loaded) {
        // Calculate scale to make coffin about 2.5 units long
        float targetSize = 2.5f;
        float dimX = tombMesh.boundsMax.x - tombMesh.boundsMin.x;
        float dimY = tombMesh.boundsMax.y - tombMesh.boundsMin.y;
        float dimZ = tombMesh.boundsMax.z - tombMesh.boundsMin.z;
        float modelLength = dimX;
        if (dimY > modelLength) modelLength = dimY;
        if (dimZ > modelLength) modelLength = dimZ;
        float coffinScale = targetSize / std::max(0.001f, modelLength);
        
        glPushMatrix();
        // Rotate to lay flat (coffin model is likely standing up)
        glRotatef(-90.0f, 1, 0, 0);  // Rotate around X to lay it down
        glScalef(coffinScale, coffinScale, coffinScale);
        glTranslatef(-tombMesh.boundsCenter.x, -tombMesh.boundsCenter.y, -tombMesh.boundsMin.z);
        
        // Bind coffin texture
        if (tombTexture.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, tombTexture.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        } else {
            glColor3f(0.8f, 0.7f, 0.5f); // Golden/sandy color fallback
        }
        
        // Render coffin mesh
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < tombMesh.indices.size(); i++) {
            unsigned int idx = tombMesh.indices[i];
            if (idx * 3 + 2 < tombMesh.normals.size()) {
                glNormal3f(tombMesh.normals[idx * 3], tombMesh.normals[idx * 3 + 1], tombMesh.normals[idx * 3 + 2]);
            }
            if (idx * 2 + 1 < tombMesh.texcoords.size()) {
                glTexCoord2f(tombMesh.texcoords[idx * 2], tombMesh.texcoords[idx * 2 + 1]);
            }
            if (idx * 3 + 2 < tombMesh.positions.size()) {
                glVertex3f(tombMesh.positions[idx * 3], tombMesh.positions[idx * 3 + 1], tombMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    } else {
        // Fallback to procedural sarcophagus
        float length = 3.0f;
        float width = 1.5f;
        float height = 1.2f;
        
        if (tombTexture.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, tombTexture.id);
            glColor3f(0.9f, 0.85f, 0.7f);
        } else {
            glColor3f(0.7f, 0.6f, 0.4f);
        }
        
        float hl = length * 0.5f;
        float hw = width * 0.5f;
        
        glBegin(GL_QUADS);
        // Top
        glNormal3f(0, 1, 0);
        glTexCoord2f(0, 0); glVertex3f(-hl, height, -hw);
        glTexCoord2f(1, 0); glVertex3f(hl, height, -hw);
        glTexCoord2f(1, 1); glVertex3f(hl, height, hw);
        glTexCoord2f(0, 1); glVertex3f(-hl, height, hw);
        // Front
        glNormal3f(0, 0, 1);
        glTexCoord2f(0, 0); glVertex3f(-hl, 0, hw);
        glTexCoord2f(1, 0); glVertex3f(hl, 0, hw);
        glTexCoord2f(1, 0.5f); glVertex3f(hl, height, hw);
        glTexCoord2f(0, 0.5f); glVertex3f(-hl, height, hw);
        // Back
        glNormal3f(0, 0, -1);
        glTexCoord2f(0, 0.5f); glVertex3f(-hl, height, -hw);
        glTexCoord2f(1, 0.5f); glVertex3f(hl, height, -hw);
        glTexCoord2f(1, 0); glVertex3f(hl, 0, -hw);
        glTexCoord2f(0, 0); glVertex3f(-hl, 0, -hw);
        // Left
        glNormal3f(-1, 0, 0);
        glTexCoord2f(0, 0); glVertex3f(-hl, 0, -hw);
        glTexCoord2f(0, 0.5f); glVertex3f(-hl, height, -hw);
        glTexCoord2f(0.5f, 0.5f); glVertex3f(-hl, height, hw);
        glTexCoord2f(0.5f, 0); glVertex3f(-hl, 0, hw);
        // Right
        glNormal3f(1, 0, 0);
        glTexCoord2f(0.5f, 0); glVertex3f(hl, 0, hw);
        glTexCoord2f(0.5f, 0.5f); glVertex3f(hl, height, hw);
        glTexCoord2f(0, 0.5f); glVertex3f(hl, height, -hw);
        glTexCoord2f(0, 0); glVertex3f(hl, 0, -hw);
        glEnd();
        
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }
    
    glPopMatrix();
}

void drawMovingPlatform(Vector3f pos, float offset) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y + offset, pos.z);

    glColor3f(0.5f, 0.4f, 0.3f);
    drawCube(2.0f);
    glPopMatrix();
}

void drawGuardianStatue(Vector3f pos, float rotation) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotation, 0, 1, 0);

    // Use Anubis OBJ model if loaded
    if (anubisMesh.loaded) {
        // Calculate scale to make Anubis about 3.0 units tall (statue size)
        float targetSize = 3.0f;
        float modelHeight = anubisMesh.boundsMax.y - anubisMesh.boundsMin.y;
        float anubisScale = targetSize / std::max(0.001f, modelHeight);
        
        glPushMatrix();
        glScalef(anubisScale, anubisScale, anubisScale);
        glTranslatef(-anubisMesh.boundsCenter.x, -anubisMesh.boundsMin.y, -anubisMesh.boundsCenter.z);
        
        bool hasUV = !anubisMesh.texcoords.empty();
        bool hasNormals = !anubisMesh.normals.empty();
        
        if (anubisMesh.albedo.valid()) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, anubisMesh.albedo.id);
            glColor3f(1.0f, 1.0f, 1.0f);
        } else {
            glDisable(GL_TEXTURE_2D);
            glColor3f(0.6f, 0.5f, 0.3f);  // Sandy gold color
        }
        
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < anubisMesh.indices.size(); i++) {
            unsigned int idx = anubisMesh.indices[i];
            if (hasUV && idx * 2 + 1 < anubisMesh.texcoords.size()) {
                glTexCoord2f(anubisMesh.texcoords[idx * 2], anubisMesh.texcoords[idx * 2 + 1]);
            }
            if (hasNormals && idx * 3 + 2 < anubisMesh.normals.size()) {
                glNormal3f(anubisMesh.normals[idx * 3], anubisMesh.normals[idx * 3 + 1], anubisMesh.normals[idx * 3 + 2]);
            }
            if (idx * 3 + 2 < anubisMesh.positions.size()) {
                glVertex3f(anubisMesh.positions[idx * 3], anubisMesh.positions[idx * 3 + 1], anubisMesh.positions[idx * 3 + 2]);
            }
        }
        glEnd();
        
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    } else {
        // Fallback: Draw placeholder jackal-headed statue
        glColor3f(0.6f, 0.5f, 0.4f);
        
        // Body
        glPushMatrix();
        glTranslatef(0, 1.5f, 0);
        drawCube(1.0f);
        glPopMatrix();

        // Head (jackal-like)
        glPushMatrix();
        glTranslatef(0, 2.5f, 0);
        glColor3f(0.4f, 0.35f, 0.25f);
        drawSphere(0.4f, 16, 16);
        // Snout
        glPushMatrix();
        glTranslatef(0, -0.1f, 0.4f);
        glScalef(0.3f, 0.2f, 0.5f);
        drawCube(1.0f);
        glPopMatrix();
        glPopMatrix();

        // Arms
        glColor3f(0.6f, 0.5f, 0.4f);
        glPushMatrix();
        glTranslatef(-0.7f, 1.5f, 0);
        drawCylinder(0.15f, 0.15f, 1.0f, 12, 1);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.7f, 1.5f, 0);
        drawCylinder(0.15f, 0.15f, 1.0f, 12, 1);
        glPopMatrix();
    }

    glPopMatrix();
}

void drawNeoTokyoEnvironment() {
    // Floor + walls texture with camera-occlusion fade on walls
    float floorRepeat = 8.0f;
    useTexture(neoTokyoSurface);

    // Floor (no fade needed)
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, floorRepeat); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glTexCoord2f(0, floorRepeat); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glEnd();

    // Wall fade based on camera proximity/intersection
    float wallRepeat = 6.0f;
    float fade = 1.0f;
    if (camera.mode == CAMERA_THIRD_PERSON) {
        const float fadeDist = 1.5f;
        if (camera.eye.z < WORLD_MIN + fadeDist) fade = 0.35f;
        if (camera.eye.z > WORLD_MAX - fadeDist) fade = std::min(fade, 0.35f);
        if (camera.eye.x > WORLD_MAX - fadeDist) fade = std::min(fade, 0.35f);
        if (camera.eye.x < WORLD_MIN + fadeDist) fade = std::min(fade, 0.35f);
    }
    bool useFade = fade < 0.99f;
    if (useFade) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(1.0f, 1.0f, 1.0f, fade);
    }

    // North wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.2f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MIN);
    glTexCoord2f(0, CEILING_Y * 0.2f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MIN);
    glEnd();
    // South wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glTexCoord2f(0, CEILING_Y * 0.2f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.2f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glEnd();
    // East wall
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.2f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MAX);
    glTexCoord2f(0, CEILING_Y * 0.2f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MIN);
    glEnd();
    // West wall
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(0, CEILING_Y * 0.2f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.2f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glEnd();

    if (useFade) {
        glDisable(GL_BLEND);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Decorative panels/screens (custom meshes)
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, neoTokyoSurface.id);
    glColor3f(0.8f, 0.9f, 1.0f);
    auto drawPanel = [](float w, float h) {
        float hw = w * 0.5f;
        float hh = h * 0.5f;
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glTexCoord2f(0, 0); glVertex3f(-hw, -hh, 0);
        glTexCoord2f(1, 0); glVertex3f(hw, -hh, 0);
        glTexCoord2f(1, 1); glVertex3f(hw, hh, 0);
        glTexCoord2f(0, 1); glVertex3f(-hw, hh, 0);
        glEnd();
    };
    // Place panels on walls
    struct Panel { float x, y, z, yaw, w, h; };
    Panel panels[] = {
        { -10.0f, 4.0f, WORLD_MIN + 0.1f,   0.0f, 6.0f, 3.0f },
        {  10.0f, 4.0f, WORLD_MIN + 0.1f,   0.0f, 5.0f, 2.5f },
        { WORLD_MIN + 0.1f, 3.5f, -10.0f,  90.0f, 4.5f, 2.0f },
        { WORLD_MIN + 0.1f, 5.0f,  10.0f,  90.0f, 5.5f, 2.5f },
        {  15.0f, 3.0f, WORLD_MAX - 0.1f, 180.0f, 6.5f, 3.0f },
        { -15.0f, 5.0f, WORLD_MAX - 0.1f, 180.0f, 5.0f, 2.5f }
    };
    for (auto& p : panels) {
        glPushMatrix();
        glTranslatef(p.x, p.y, p.z);
        glRotatef(p.yaw, 0, 1, 0);
        drawPanel(p.w, p.h);
        glPopMatrix();
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // =============================================================================
    // NEO-TOKYO NEON SIGNS (Japanese neon street signs on walls)
    // =============================================================================
    // Place neon signs at various locations on the walls
    drawNeonSign(Vector3f(0.0f, 6.0f, WORLD_MIN + 0.2f), 0.0f, 1.5f);      // North wall center
    drawNeonSign(Vector3f(-20.0f, 5.0f, WORLD_MIN + 0.2f), 0.0f, 1.2f);    // North wall left
    drawNeonSign(Vector3f(WORLD_MAX - 0.2f, 7.0f, 0.0f), -90.0f, 1.3f);    // East wall center
    drawNeonSign(Vector3f(WORLD_MIN + 0.2f, 6.0f, 5.0f), 90.0f, 1.2f);     // West wall
    drawNeonSign(Vector3f(5.0f, 5.5f, WORLD_MAX - 0.2f), 180.0f, 1.4f);    // South wall

    // =============================================================================
    // NEO-TOKYO STATIC MESH DECORATIONS
    // =============================================================================
    // Futuristic server racks / data terminals along walls
    glColor3f(0.15f, 0.2f, 0.25f);
    for (int i = 0; i < 4; i++) {
        float x = -20.0f + i * 12.0f;
        glPushMatrix();
        glTranslatef(x, 0, WORLD_MIN + 2.0f);
        glScalef(1.5f, 4.0f, 1.0f);
        drawCube(1.0f);
        glPopMatrix();
        // LED indicator lights
        glDisable(GL_LIGHTING);
        float blink = (sin(gameTime * 5.0f + i) > 0) ? 1.0f : 0.3f;
        glColor3f(0.0f, blink, 0.0f);
        glPushMatrix();
        glTranslatef(x, 3.0f, WORLD_MIN + 1.6f);
        drawSphere(0.1f, 8, 8);
        glPopMatrix();
        glEnable(GL_LIGHTING);
        glColor3f(0.15f, 0.2f, 0.25f);
    }

        // Draw collectibles
        for (size_t i = 0; i < crystals.size(); i++) {
            if (!crystals[i].collected) {
                crystals[i].update();
                drawTemporalCrystal(crystals[i].position, crystals[i].rotation, crystals[i].bobOffset);
            }
        }
        
        // Draw pickup animations
        for (size_t i = 0; i < pickupAnimations.size(); i++) {
            if (pickupAnimations[i].active && pickupAnimations[i].type == 0) {
                float animTime = gameTime - pickupAnimations[i].time;
                if (animTime < 0.5f) {
                    float scale = 1.0f + sin(animTime * M_PI * 4) * 0.5f;
                    float height = animTime * 3.0f;
                    float rotation = animTime * 720.0f;
                    glPushMatrix();
                    glTranslatef(pickupAnimations[i].position.x, 
                                pickupAnimations[i].position.y + height, 
                                pickupAnimations[i].position.z);
                    glScalef(scale, scale, scale);
                    glRotatef(rotation, 0, 1, 0);
                    drawTemporalCrystal(Vector3f(0, 0, 0), rotation, 0);
                    glPopMatrix();
                } else {
                    pickupAnimations[i].active = false;
                }
            }
        }

    // Draw obstacles and interactive objects
    // Laser Grid
    bool laserActive = (int)(gameTime * 2) % 2 == 0;
    drawLaserGrid(Vector3f(10, 2, 0), 4.0f, laserActive);

    // Security Camera
    float camRotation = fmod(gameTime * 30.0f, 360.0f);
    float sweepAngle = alarmActive ? 120.0f : 60.0f;
    drawSecurityCamera(Vector3f(-15, 5, -15), camRotation, sweepAngle);

    // Control Console
    drawControlConsole(Vector3f(-20, 1, -20), consoleActivated);

    // Security Robot
    drawSecurityRobot(securityRobot);

    // Exit Portal
    float portalPulse = sin(gameTime * 3.0f) * 0.5f + 0.5f;
    drawExitPortal(Vector3f(20, 2, 20), portalUnlocked, portalPulse);
}

void drawTempleEnvironment() {
    useTexture(templeSurface);
    float floorRepeat = 6.0f;
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, floorRepeat); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glTexCoord2f(0, floorRepeat); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glEnd();

    // Walls with hieroglyphic texture pattern (with camera fade)
    float wallRepeat = 4.0f;
    float fade = 1.0f;
    if (camera.mode == CAMERA_THIRD_PERSON) {
        const float fadeDist = 1.5f;
        if (camera.eye.z < WORLD_MIN + fadeDist) fade = 0.35f;
        if (camera.eye.z > WORLD_MAX - fadeDist) fade = std::min(fade, 0.35f);
        if (camera.eye.x > WORLD_MAX - fadeDist) fade = std::min(fade, 0.35f);
        if (camera.eye.x < WORLD_MIN + fadeDist) fade = std::min(fade, 0.35f);
    }
    bool useFade = fade < 0.99f;
    if (useFade) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(1.0f, 1.0f, 1.0f, fade);
    }
    // North wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.15f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MIN);
    glTexCoord2f(0, CEILING_Y * 0.15f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MIN);
    glEnd();
    // South wall
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glTexCoord2f(0, CEILING_Y * 0.15f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.15f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glEnd();
    // East wall
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.15f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MAX);
    glTexCoord2f(0, CEILING_Y * 0.15f); glVertex3f(WORLD_MAX, CEILING_Y, WORLD_MIN);
    glEnd();
    // West wall
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(0, CEILING_Y * 0.15f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MIN);
    glTexCoord2f(wallRepeat, CEILING_Y * 0.15f); glVertex3f(WORLD_MIN, CEILING_Y, WORLD_MAX);
    glTexCoord2f(wallRepeat, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glEnd();
    if (useFade) {
        glDisable(GL_BLEND);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // =============================================================================
    // TALL EGYPTIAN COLUMNS AT ALL 4 CORNERS
    // =============================================================================
    float columnHeight = 12.0f;  // Relatively tall columns
    float cornerOffset = 5.0f;   // Distance from corner
    drawTallColumn(Vector3f(WORLD_MIN + cornerOffset, GROUND_Y, WORLD_MIN + cornerOffset), columnHeight);
    drawTallColumn(Vector3f(WORLD_MAX - cornerOffset, GROUND_Y, WORLD_MIN + cornerOffset), columnHeight);
    drawTallColumn(Vector3f(WORLD_MIN + cornerOffset, GROUND_Y, WORLD_MAX - cornerOffset), columnHeight);
    drawTallColumn(Vector3f(WORLD_MAX - cornerOffset, GROUND_Y, WORLD_MAX - cornerOffset), columnHeight);

    // Original ancient columns (ring in center) - REMOVED per user request
    // Keeping only the 4 corner columns

    // Pyramids (decorative) - using temple texture, larger size, darker color
    useTexture(templeSurface);
    for (int i = 0; i < 4; i++) {
        float x = -20.0f + (i % 2) * 40.0f;
        float z = -20.0f + (i / 2) * 40.0f;
        glPushMatrix();
        glTranslatef(x, 0.0f, z);  // Base at ground level
        glColor3f(0.5f, 0.5f, 0.5f);  // Darker color to make texture appear darker
        drawPyramid(5.0f, 6.0f);  // Larger: base size 5.0, height 6.0
        glPopMatrix();
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // =============================================================================
    // HIEROGLYPHIC WALL PANELS - Cover all walls completely
    // =============================================================================
    // North wall (Z = WORLD_MIN) - Multiple panels to cover entire wall
    drawHieroglyphicWall(Vector3f(-25.0f, GROUND_Y, WORLD_MIN + 0.3f), 0.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(-10.0f, GROUND_Y, WORLD_MIN + 0.3f), 0.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(5.0f, GROUND_Y, WORLD_MIN + 0.3f), 0.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(20.0f, GROUND_Y, WORLD_MIN + 0.3f), 0.0f, 12.0f, 15.0f);
    
    // South wall (Z = WORLD_MAX) - Multiple panels
    drawHieroglyphicWall(Vector3f(-25.0f, GROUND_Y, WORLD_MAX - 0.3f), 180.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(-10.0f, GROUND_Y, WORLD_MAX - 0.3f), 180.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(5.0f, GROUND_Y, WORLD_MAX - 0.3f), 180.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(20.0f, GROUND_Y, WORLD_MAX - 0.3f), 180.0f, 12.0f, 15.0f);
    
    // East wall (X = WORLD_MAX) - Multiple panels
    drawHieroglyphicWall(Vector3f(WORLD_MAX - 0.3f, GROUND_Y, -25.0f), -90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MAX - 0.3f, GROUND_Y, -10.0f), -90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MAX - 0.3f, GROUND_Y, 5.0f), -90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MAX - 0.3f, GROUND_Y, 20.0f), -90.0f, 12.0f, 15.0f);
    
    // West wall (X = WORLD_MIN) - Multiple panels
    drawHieroglyphicWall(Vector3f(WORLD_MIN + 0.3f, GROUND_Y, -25.0f), 90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MIN + 0.3f, GROUND_Y, -10.0f), 90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MIN + 0.3f, GROUND_Y, 5.0f), 90.0f, 12.0f, 15.0f);
    drawHieroglyphicWall(Vector3f(WORLD_MIN + 0.3f, GROUND_Y, 20.0f), 90.0f, 12.0f, 15.0f);

    // =============================================================================
    // SARCOPHAGI (2 tomb coffins)
    // =============================================================================
    drawSarcophagus(Vector3f(-8.0f, GROUND_Y, 15.0f), 45.0f);   // First sarcophagus
    drawSarcophagus(Vector3f(12.0f, GROUND_Y, -8.0f), -30.0f);  // Second sarcophagus

    // Draw collectibles
    for (size_t i = 0; i < scarabs.size(); i++) {
        if (!scarabs[i].collected) {
            scarabs[i].update();
            drawGoldenScarab(scarabs[i].position, scarabs[i].rotation, scarabs[i].bobOffset);
        }
    }
    
    // Draw pickup animations
    for (size_t i = 0; i < pickupAnimations.size(); i++) {
        if (pickupAnimations[i].active && pickupAnimations[i].type == 1) {
            float animTime = gameTime - pickupAnimations[i].time;
            if (animTime < 0.5f) {
                float scale = 1.0f + sin(animTime * M_PI * 4) * 0.5f;
                float height = animTime * 3.0f;
                float rotation = animTime * 720.0f;
                glPushMatrix();
                glTranslatef(pickupAnimations[i].position.x, 
                            pickupAnimations[i].position.y + height, 
                            pickupAnimations[i].position.z);
                glScalef(scale, scale, scale);
                glRotatef(rotation, 0, 1, 0);
                drawGoldenScarab(Vector3f(0, 0, 0), rotation, 0);
                glPopMatrix();
            } else {
                pickupAnimations[i].active = false;
            }
        }
    }

    // =============================================================================
    // PRESSURE PLATE TRAP WITH TRAP DOOR AND SPIKES
    // =============================================================================
    drawPressurePlateTrap(templePressurePlate);

    // =============================================================================
    // MOVING PLATFORM OVER FATAL PIT
    // =============================================================================
    drawMovingPlatformWithPit(templeMovingPlatform);

    // =============================================================================
    // GUARDIAN STATUE WITH PATROL (uses templeGuardian)
    // =============================================================================
    drawGuardianStatue(templeGuardian.position, templeGuardian.yaw);

    // Exit Portal (moved inward to avoid corner pyramid)
    bool portalReady = player.scarabsCollected >= SCARABS_REQUIRED;
    float portalPulse = sin(gameTime * 3.0f) * 0.5f + 0.5f;
    drawExitPortal(Vector3f(-12, 2, -20), portalReady, portalPulse);
}

void drawHUD() {
    // Switch to 2D orthographic projection
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    // Score
    char scoreText[64];
    sprintf(scoreText, "Score: %d", player.score);
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(10, WINDOW_HEIGHT - 30);
    for (int i = 0; scoreText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, scoreText[i]);
    }
    
    // Health bar
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_QUADS);
    glVertex2f(10, WINDOW_HEIGHT - 60);
    glVertex2f(10 + (player.health / 100.0f) * 200, WINDOW_HEIGHT - 60);
    glVertex2f(10 + (player.health / 100.0f) * 200, WINDOW_HEIGHT - 80);
    glVertex2f(10, WINDOW_HEIGHT - 80);
    glEnd();
    
    glColor3f(1.0f, 1.0f, 1.0f);
    char healthText[64];
    sprintf(healthText, "Health: %d", player.health);
    glRasterPos2f(10, WINDOW_HEIGHT - 100);
    for (int i = 0; healthText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, healthText[i]);
    }
    
    // Collected items
    char itemsText[64];
    if (gameState == STATE_NEO_TOKYO) {
        sprintf(itemsText, "Crystals: %d/%d", player.crystalsCollected, MAX_CRYSTALS);
    } else {
        sprintf(itemsText, "Scarabs: %d/%d", player.scarabsCollected, MAX_SCARABS);
    }
    glRasterPos2f(10, WINDOW_HEIGHT - 130);
    for (int i = 0; itemsText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, itemsText[i]);
    }
    
    // Objective hint
    char objectiveText[128];
    if (gameState == STATE_NEO_TOKYO) {
        if (!consoleActivated) {
            sprintf(objectiveText, "Objective: Activate Control Console");
        } else if (!portalUnlocked) {
            sprintf(objectiveText, "Objective: Reach Exit Portal");
        } else {
            sprintf(objectiveText, "Objective: Enter Portal");
        }
    } else {
        if (player.scarabsCollected < SCARABS_REQUIRED) {
            sprintf(objectiveText, "Objective: Collect %d Scarabs (%d/%d)", 
                    SCARABS_REQUIRED, player.scarabsCollected, SCARABS_REQUIRED);
        } else {
            sprintf(objectiveText, "Objective: Reach Exit Portal");
        }
    }
    glRasterPos2f(10, WINDOW_HEIGHT - 160);
    for (int i = 0; objectiveText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, objectiveText[i]);
    }
    
    // Alarm indicator (Neo-Tokyo)
    if (gameState == STATE_NEO_TOKYO && alarmActive) {
        float pulse = sin(gameTime * 10.0f) * 0.5f + 0.5f;
        glColor4f(1.0f, 0.0f, 0.0f, 0.3f + pulse * 0.4f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_QUADS);
        glVertex2f(0, 0);
        glVertex2f(WINDOW_WIDTH, 0);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT);
        glVertex2f(0, WINDOW_HEIGHT);
        glEnd();
        glDisable(GL_BLEND);
        
        glColor3f(1.0f, 0.0f, 0.0f);
        char alarmText[] = "ALARM ACTIVE!";
        glRasterPos2f(WINDOW_WIDTH / 2 - 60, WINDOW_HEIGHT / 2);
        for (int i = 0; alarmText[i] != '\0'; i++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, alarmText[i]);
        }
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawMenu() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(1.0f, 1.0f, 1.0f);
    char title[] = "CHRONO HEIST";
    glRasterPos2f(WINDOW_WIDTH / 2 - 100, WINDOW_HEIGHT / 2 + 50);
    for (int i = 0; title[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, title[i]);
    }
    
    char start[] = "Press SPACE to Start";
    glRasterPos2f(WINDOW_WIDTH / 2 - 120, WINDOW_HEIGHT / 2);
    for (int i = 0; start[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, start[i]);
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawWinScreen() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(0.0f, 1.0f, 0.0f);
    char winText[] = "GAME WIN!";
    glRasterPos2f(WINDOW_WIDTH / 2 - 80, WINDOW_HEIGHT / 2);
    for (int i = 0; winText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, winText[i]);
    }
    
    char scoreText[64];
    sprintf(scoreText, "Final Score: %d", player.score);
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(WINDOW_WIDTH / 2 - 100, WINDOW_HEIGHT / 2 - 40);
    for (int i = 0; scoreText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, scoreText[i]);
    }
    
    // Restart prompt
    glColor3f(0.8f, 0.8f, 0.8f);
    char restartText[] = "Press R to Restart";
    glRasterPos2f(WINDOW_WIDTH / 2 - 100, WINDOW_HEIGHT / 2 - 80);
    for (int i = 0; restartText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, restartText[i]);
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawLoseScreen() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    char loseText[] = "GAME LOSE!";
    glRasterPos2f(WINDOW_WIDTH / 2 - 80, WINDOW_HEIGHT / 2);
    for (int i = 0; loseText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, loseText[i]);
    }
    
    // Restart prompt
    glColor3f(0.8f, 0.8f, 0.8f);
    char restartText[] = "Press R to Restart";
    glRasterPos2f(WINDOW_WIDTH / 2 - 100, WINDOW_HEIGHT / 2 - 80);
    for (int i = 0; restartText[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, restartText[i]);
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

// =============================================================================
// Game Logic Functions
// =============================================================================
void initializeNeoTokyo() {
    crystals.clear();
    pickupAnimations.clear();
    // Place 6 Temporal Crystals
    crystals.push_back(Collectible(Vector3f(-15, 1.5f, -15), 0));
    crystals.push_back(Collectible(Vector3f(15, 1.5f, -15), 0));
    crystals.push_back(Collectible(Vector3f(-15, 1.5f, 15), 0));
    crystals.push_back(Collectible(Vector3f(15, 1.5f, 15), 0));
    crystals.push_back(Collectible(Vector3f(0, 1.5f, -20), 0));
    crystals.push_back(Collectible(Vector3f(0, 1.5f, 20), 0));
    
    // Reset player position and orientation (spawn at ground level)
    player.position = Vector3f(0, GROUND_Y + PLAYER_HEIGHT * 0.5f, 0);
    player.velocity = Vector3f(0, 0, 0);
    player.yaw = 0.0f;
    player.targetYaw = 0.0f;
    player.pitch = 0.0f;
    player.crystalsCollected = 0;
    
    // Reset game state
    consoleActivated = false;
    portalUnlocked = false;
    alarmActive = false;
    hitReactionActive = false;
    consoleActivationAnimActive = false;
    portalActivationAnimActive = false;
    lighting.currentScene = 0;
    
    // Reset security robot
    securityRobot.position = Vector3f(15.0f, GROUND_Y + 1.0f, 15.0f);
    securityRobot.velocity = Vector3f(0, 0, 0);
    securityRobot.yaw = 0.0f;
    securityRobot.damageTimer = 0.0f;
    securityRobot.active = false;
    
    // Reset camera (keep current mode, reset angles)
    camera.cameraYaw = 0.0f;
    camera.cameraPitch = 20.0f;
    camera.update(player);
}

void initializeTemple() {
    scarabs.clear();
    pickupAnimations.clear();
    // Place 6 Golden Scarabs
    scarabs.push_back(Collectible(Vector3f(-18, 1.5f, -18), 1));
    scarabs.push_back(Collectible(Vector3f(18, 1.5f, -18), 1));
    scarabs.push_back(Collectible(Vector3f(-18, 1.5f, 18), 1));
    scarabs.push_back(Collectible(Vector3f(18, 1.5f, 18), 1));
    scarabs.push_back(Collectible(Vector3f(0, 1.5f, -25), 1));
    scarabs.push_back(Collectible(Vector3f(0, 1.5f, 25), 1));
    
    // Reset player position and orientation (spawn at ground level)
    player.position = Vector3f(0, GROUND_Y + PLAYER_HEIGHT * 0.5f, 0);
    player.velocity = Vector3f(0, 0, 0);
    player.yaw = 0.0f;
    player.targetYaw = 0.0f;
    player.pitch = 0.0f;
    player.scarabsCollected = 0;
    
    // Reset game state
    hitReactionActive = false;
    consoleActivationAnimActive = false;
    portalActivationAnimActive = false;
    lighting.currentScene = 1;
    
    // Reset pressure plate trap
    templePressurePlate = PressurePlateTrap(Vector3f(5.0f, GROUND_Y, 0.0f));
    
    // Reset moving platform
    templeMovingPlatform = MovingPlatform();
    
    // Reset guardian patrol
    templeGuardian = GuardianPatrol();
    
    // Reset camera (keep current mode, reset angles)
    camera.cameraYaw = 0.0f;
    camera.cameraPitch = 20.0f;
    camera.update(player);
}

void checkCollectibles() {
    if (gameState == STATE_NEO_TOKYO) {
        for (size_t i = 0; i < crystals.size(); i++) {
            if (crystals[i].checkCollision(player.position, PLAYER_RADIUS)) {
                if (!crystals[i].collected) {
                    crystals[i].collected = true;
                    player.crystalsCollected++;
                    player.score += CRYSTAL_POINTS;
                    playSFX("pickup_crystal");
                    
                    // Trigger pickup animation
                    PickupAnimation anim;
                    anim.position = crystals[i].position;
                    anim.time = gameTime;
                    anim.active = true;
                    anim.type = 0;
                    pickupAnimations.push_back(anim);
                }
            }
        }
    } else if (gameState == STATE_TEMPLE) {
        for (size_t i = 0; i < scarabs.size(); i++) {
            if (scarabs[i].checkCollision(player.position, PLAYER_RADIUS)) {
                if (!scarabs[i].collected) {
                    scarabs[i].collected = true;
                    player.scarabsCollected++;
                    player.score += SCARAB_POINTS;
                    playSFX("pickup_scarab");
                    
                    // Trigger pickup animation
                    PickupAnimation anim;
                    anim.position = scarabs[i].position;
                    anim.time = gameTime;
                    anim.active = true;
                    anim.type = 1;
                    pickupAnimations.push_back(anim);
                }
            }
        }
    }
}

void checkObstacles(float deltaTime) {
    if (gameState == STATE_NEO_TOKYO) {
        // Laser Grid collision
        Vector3f laserPos(10, 2, 0);
        bool laserActive = (int)(gameTime * 2) % 2 == 0;
        if (laserActive) {
            Vector3f diff = player.position - laserPos;
            if (diff.lengthSquared() < 16.0f) {
                player.health -= 1;
                hitReactionActive = true;
                hitReactionTime = gameTime;
                playSFX("laser_hit");
            }
        }

        // Security Camera detection
        Vector3f camPos(-15, 5, -15);
        Vector3f toPlayer = player.position - camPos;
        float dist = toPlayer.length();
        if (dist < 10.0f) {
            float camRotation = fmod(gameTime * 30.0f, 360.0f);
            float angleToPlayer = RAD2DEG(atan2(toPlayer.x, toPlayer.z));
            float angleDiff = fabs(angleToPlayer - camRotation);
            if (angleDiff > 180.0f) angleDiff = 360.0f - angleDiff;
            
            float sweepAngle = alarmActive ? 120.0f : 60.0f;
            if (angleDiff < sweepAngle * 0.5f) {
                if (!alarmActive) {
                    alarmActive = true;
                    alarmTime = gameTime;
                    playSFX("alarm");
                }
            }
        }

        // Motion Detector
        Vector3f detectorPos(5, 1, 5);
        if ((player.position - detectorPos).lengthSquared() < 9.0f) {
            if (!alarmActive) {
                alarmActive = true;
                alarmTime = gameTime;
                player.score = (player.score > 100) ? player.score - 100 : 0;
                playSFX("motion_detected");
            }
        }

        // Alarm timeout
        if (alarmActive && (gameTime - alarmTime) > 5.0f) {
            alarmActive = false;
        }

        // Security Robot update and collision (Neo-Tokyo only)
        securityRobot.update(player.position, alarmActive, gameState == STATE_NEO_TOKYO, deltaTime);
        if (securityRobot.active && securityRobot.checkCollisionWithPlayer(player.position)) {
            if (securityRobot.damageTimer <= 0) {
                player.health -= 3;  // Low damage
                hitReactionActive = true;
                hitReactionTime = gameTime;
                securityRobot.damageTimer = 1.0f;  // 1 second cooldown between hits
                playSFX("robot_hit");
            }
        }

        // Control Console interaction (larger activation radius to match wider console model)
        Vector3f consolePos(-20, 1, -20);
        if ((player.position - consolePos).lengthSquared() < 16.0f) {  // radius ~4 units
            if (keys['E'] || keys['e']) {
                if (!consoleActivated) {
                    consoleActivated = true;
                    portalUnlocked = true;
                    // Trigger console activation animation
                    consoleActivationAnimActive = true;
                    consoleActivationTime = gameTime;
                    // Trigger portal unlock animation
                    portalActivationAnimActive = true;
                    portalActivationTime = gameTime;
                    playSFX("console_activate");
                }
            }
        }

        // Exit Portal
        Vector3f portalPos(20, 2, 20);
        if (portalUnlocked && (player.position - portalPos).lengthSquared() < 9.0f) {
            // Transition to temple
            gameState = STATE_TEMPLE;
            initializeTemple();
            playSFX("portal_enter");
        }
    } else if (gameState == STATE_TEMPLE) {
        // =============================================================================
        // PRESSURE PLATE TRAP (Single trap with trap door and spikes)
        // =============================================================================
        templePressurePlate.update(gameTime, deltaTime);
        
        // Check if player steps on the plate
        if (!templePressurePlate.triggered && templePressurePlate.checkPlayerOnPlate(player.position)) {
            templePressurePlate.triggered = true;
            templePressurePlate.triggerTime = gameTime;
            playSFX("trap_trigger");
        }
        
        // Check if player is in the pit with spikes
        if (templePressurePlate.checkPlayerInPit(player.position)) {
            templePressurePlate.playerInTrap = true;
            
            // Fall damage when landing at the bottom of the pit
            float pitBottom = GROUND_Y - PressurePlateTrap::PIT_DEPTH + 1.0f;
            if (player.position.y <= pitBottom + PLAYER_HEIGHT * 0.5f) {
                // Land at pit bottom
                player.position.y = pitBottom + PLAYER_HEIGHT * 0.5f;
                
                // Apply fall damage if falling fast
                if (player.velocity.y < -0.3f) {
                    int fallDamage = (int)(fabsf(player.velocity.y) * 30.0f);
                    player.health -= fallDamage;
                    hitReactionActive = true;
                    hitReactionTime = gameTime;
                    playSFX("fall_impact");
                }
                player.velocity.y = 0;
                player.onGround = true;
            }
            
            // Spike damage (player must jump out!)
            if (templePressurePlate.spikeHeight > 0.5f && templePressurePlate.damageTimer <= 0) {
                player.health -= PressurePlateTrap::SPIKE_DAMAGE;
                hitReactionActive = true;
                hitReactionTime = gameTime;
                templePressurePlate.damageTimer = PressurePlateTrap::DAMAGE_COOLDOWN;
                playSFX("spike_hit");
            }
        } else {
            templePressurePlate.playerInTrap = false;
        }

        // =============================================================================
        // MOVING PLATFORM OVER FATAL PIT
        // =============================================================================
        templeMovingPlatform.update(deltaTime);
        
        // Check if player is on the platform
        if (templeMovingPlatform.checkPlayerOnPlatform(player.position)) {
            // Move player with the platform
            Vector3f platPos = templeMovingPlatform.getPlatformPosition();
            player.position.x += templeMovingPlatform.direction * templeMovingPlatform.speed * deltaTime;
            player.position.y = platPos.y + 0.8f;
            player.onGround = true;
            player.velocity.y = 0;
        }
        
        // Check if player fell into the fatal pit
        if (templeMovingPlatform.checkPlayerInPit(player.position)) {
            // Fatal fall - instant death
            player.health = 0;
            playSFX("fall_death");
        }

        // =============================================================================
        // GUARDIAN STATUE PATROL (Back and forth with chase when near)
        // =============================================================================
        templeGuardian.update(player.position, deltaTime);
        
        // Guardian collision damage
        if (templeGuardian.checkCollisionWithPlayer(player.position)) {
            if (templeGuardian.damageTimer <= 0) {
                player.health -= 5;
                Vector3f knockback = (player.position - templeGuardian.position).unit() * 2.0f;
                player.velocity = player.velocity + knockback;
                hitReactionActive = true;
                hitReactionTime = gameTime;
                templeGuardian.damageTimer = 1.0f;  // 1 second cooldown
                playSFX("guardian_hit");
            }
        }

        // Exit Portal (relocated to avoid pyramid overlap)
        Vector3f portalPos(-12, 2, -20);
        if (player.scarabsCollected >= SCARABS_REQUIRED) {
            if ((player.position - portalPos).lengthSquared() < 9.0f) {
                gameState = STATE_WIN;
                playSFX("portal_enter");
            }
        }
    }
}

void updateGame(float deltaTime) {
    gameTime += deltaTime;
    
    if (gameState == STATE_MENU || gameState == STATE_WIN || gameState == STATE_LOSE) {
        return;
    }

    // Update hit reaction
    if (hitReactionActive && (gameTime - hitReactionTime) > 0.3f) {
        hitReactionActive = false;
    }

    // Update console activation animation (lasts 1.5 seconds)
    if (consoleActivationAnimActive && (gameTime - consoleActivationTime) > 1.5f) {
        consoleActivationAnimActive = false;
    }

    // Update portal activation animation (lasts 2.0 seconds)
    if (portalActivationAnimActive && (gameTime - portalActivationTime) > 2.0f) {
        portalActivationAnimActive = false;
    }

    // Clean up finished pickup animations
    for (size_t i = pickupAnimations.size(); i > 0; i--) {
        if (!pickupAnimations[i-1].active) {
            pickupAnimations.erase(pickupAnimations.begin() + i - 1);
        }
    }

    // Auto-switch to temple after 30 seconds (dummy test)
    // Removed auto-transition; player advances only via portal/level completion

    // Update lighting
    lighting.update(deltaTime);

    // =========================================
    // PLAYER MOVEMENT - Relative to Camera Direction
    // =========================================
    // W = forward in camera's facing direction
    // S = backward
    // A = strafe left relative to camera
    // D = strafe right relative to camera
    // The player model automatically rotates to face movement direction
    
    Vector3f moveDir(0, 0, 0);
    if (keys['W'] || keys['w'] || keys[GLUT_KEY_UP]) moveDir.z = 1.0f;    // Forward
    if (keys['S'] || keys['s'] || keys[GLUT_KEY_DOWN]) moveDir.z = -1.0f; // Backward
    if (keys['A'] || keys['a'] || keys[GLUT_KEY_LEFT]) moveDir.x = -1.0f; // Strafe left
    if (keys['D'] || keys['d'] || keys[GLUT_KEY_RIGHT]) moveDir.x = 1.0f; // Strafe right
    
    if (moveDir.lengthSquared() > 0.0001f) {
        if (camera.mode == CAMERA_THIRD_PERSON) {
            // Third-person: Movement is relative to camera direction
            Vector3f camForward = camera.getForwardDirection();
            Vector3f camRight = camera.getRightDirection();
            player.moveRelativeToCamera(moveDir, camForward, camRight);
        } else {
            // First-person: Movement is relative to player's look direction
            float radYaw = DEG2RAD(player.yaw);
            Vector3f forward = Vector3f(sin(radYaw), 0, cos(radYaw));
            Vector3f right = Vector3f(-cos(radYaw), 0, sin(radYaw));  // Fixed: proper right vector
            
            // In first-person, don't auto-rotate player - they face where they look
            // Just apply velocity directly
            if (moveDir.lengthSquared() > 0.0001f) {
                moveDir = moveDir.unit();
            }
            Vector3f worldMoveDir = (forward * moveDir.z + right * moveDir.x) * PLAYER_SPEED;
            player.velocity.x = worldMoveDir.x;
            player.velocity.z = worldMoveDir.z;
        }
    }
    
    if (keys[' ']) {
        player.jump();
    }

    player.update();

    // -------------------------------------------------
    // Static collision blocking (world geometry)
    // -------------------------------------------------
    auto makeBox = [](const Vector3f& center, const Vector3f& half) {
        return AABB{Vector3f(center.x - half.x, center.y - half.y, center.z - half.z),
                    Vector3f(center.x + half.x, center.y + half.y, center.z + half.z)};
    };
    std::vector<AABB> staticColliders;
    if (gameState == STATE_NEO_TOKYO) {
        // Control console (wider OBJ model - approximately 2x1.5x2 units)
        staticColliders.push_back(makeBox(Vector3f(-20.0f, 0.75f, -20.0f), Vector3f(1.5f, 0.75f, 1.5f)));
        // Exit portal
        staticColliders.push_back(makeBox(Vector3f(20.0f, 2.0f, 20.0f), Vector3f(2.0f, 3.0f, 2.0f)));
        // Laser grid
        staticColliders.push_back(makeBox(Vector3f(10.0f, 2.0f, 0.0f), Vector3f(1.0f, 2.0f, 4.0f)));
        // Security camera base
        staticColliders.push_back(makeBox(Vector3f(-15.0f, 5.0f, -15.0f), Vector3f(0.5f, 0.5f, 0.5f)));
        // Motion detector
        staticColliders.push_back(makeBox(Vector3f(5.0f, 1.0f, 5.0f), Vector3f(0.5f, 0.5f, 0.5f)));
    } else if (gameState == STATE_TEMPLE) {
        // Pyramids (corners)
        staticColliders.push_back(makeBox(Vector3f(-20.0f, 3.0f, -20.0f), Vector3f(2.5f, 3.0f, 2.5f)));
        staticColliders.push_back(makeBox(Vector3f(20.0f, 3.0f, -20.0f), Vector3f(2.5f, 3.0f, 2.5f)));
        staticColliders.push_back(makeBox(Vector3f(-20.0f, 3.0f, 20.0f), Vector3f(2.5f, 3.0f, 2.5f)));
        staticColliders.push_back(makeBox(Vector3f(20.0f, 3.0f, 20.0f), Vector3f(2.5f, 3.0f, 2.5f)));
        
        // Tall columns at corners
        float columnOffset = 5.0f;
        staticColliders.push_back(makeBox(Vector3f(WORLD_MIN + columnOffset, 6.0f, WORLD_MIN + columnOffset), Vector3f(1.0f, 6.0f, 1.0f)));
        staticColliders.push_back(makeBox(Vector3f(WORLD_MAX - columnOffset, 6.0f, WORLD_MIN + columnOffset), Vector3f(1.0f, 6.0f, 1.0f)));
        staticColliders.push_back(makeBox(Vector3f(WORLD_MIN + columnOffset, 6.0f, WORLD_MAX - columnOffset), Vector3f(1.0f, 6.0f, 1.0f)));
        staticColliders.push_back(makeBox(Vector3f(WORLD_MAX - columnOffset, 6.0f, WORLD_MAX - columnOffset), Vector3f(1.0f, 6.0f, 1.0f)));
        
        // Sarcophagi (2 tombs)
        staticColliders.push_back(makeBox(Vector3f(-8.0f, 0.6f, 15.0f), Vector3f(1.8f, 0.8f, 1.0f)));
        staticColliders.push_back(makeBox(Vector3f(12.0f, 0.6f, -8.0f), Vector3f(1.8f, 0.8f, 1.0f)));
        
        // Guardian statue (now uses templeGuardian position - dynamic)
        staticColliders.push_back(makeBox(templeGuardian.position, Vector3f(1.0f, 2.0f, 1.0f)));
        
        // Exit portal (relocated to avoid pyramid overlap)
        staticColliders.push_back(makeBox(Vector3f(-12.0f, 2.0f, -20.0f), Vector3f(2.0f, 3.0f, 2.0f)));
    }

    for (const auto& box : staticColliders) {
        resolvePlayerAABBCollision(player, box);
    }
 
    // Update camera
    camera.update(player);

    // Check collisions and interactions
    checkCollectibles();
    checkObstacles(deltaTime);

    // Check win/lose conditions
    if (player.health <= 0) {
        gameState = STATE_LOSE;
        playSFX("game_over");
    }

    // Check fatal pit (Temple) - only moving platform pit is fatal, not pressure plate trap
    if (gameState == STATE_TEMPLE && player.position.y < -10.0f) {
        gameState = STATE_LOSE;
        playSFX("game_over");
    }
}

// =============================================================================
// GLUT Callbacks
// =============================================================================
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(70.0, (double)WINDOW_WIDTH / WINDOW_HEIGHT, 0.1, 200.0);  // Wider FOV, longer view distance
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    if (gameState == STATE_MENU) {
        drawMenu();
    } else if (gameState == STATE_WIN) {
        drawWinScreen();
    } else if (gameState == STATE_LOSE) {
        drawLoseScreen();
    } else {
        // Setup lighting
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_NORMALIZE);
        lighting.apply();
        
        // Setup camera
        camera.look();
        
        // Ensure proper rendering state
        glShadeModel(GL_SMOOTH);
        
        // Draw environment
        if (gameState == STATE_NEO_TOKYO) {
            drawNeoTokyoEnvironment();
        } else if (gameState == STATE_TEMPLE) {
            drawTempleEnvironment();
        }
        
        // Draw player with hit reaction
        if (hitReactionActive) {
            float flash = sin((gameTime - hitReactionTime) * 20.0f) * 0.5f + 0.5f;
            glColor3f(1.0f, flash * 0.3f, flash * 0.3f);
        }
        drawPlayer();
        glColor3f(1.0f, 1.0f, 1.0f);
        
        // Draw HUD
        drawHUD();
    }
    
    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
}

void keyboard(unsigned char key, int x, int y) {
    keys[key] = true;
    
    // Restart from win/lose screens
    if ((gameState == STATE_WIN || gameState == STATE_LOSE) && (key == 'r' || key == 'R')) {
        gameState = STATE_NEO_TOKYO;
        player.health = MAX_HEALTH;
        player.score = 0;
        gameTime = 0.0f;
        initializeNeoTokyo();
        return;
    }
    
    if (key == 'c' || key == 'C') {
        // Sync player/camera orientation when switching modes
        if (camera.mode == CAMERA_THIRD_PERSON) {
            // Switching TO first-person: Set player yaw to face camera direction
            // Player should look in the direction the camera was facing
            player.yaw = camera.cameraYaw + 180.0f;  // Opposite of camera orbit angle
            player.targetYaw = player.yaw;
            while (player.yaw > 360.0f) player.yaw -= 360.0f;
            while (player.yaw < 0.0f) player.yaw += 360.0f;
            player.pitch = 0.0f;  // Reset pitch
        } else {
            // Switching TO third-person: Set camera to orbit behind player
            camera.cameraYaw = player.yaw + 180.0f;  // Camera behind player
            while (camera.cameraYaw > 360.0f) camera.cameraYaw -= 360.0f;
            while (camera.cameraYaw < 0.0f) camera.cameraYaw += 360.0f;
            camera.cameraPitch = 20.0f;  // Reset pitch
        }
        camera.toggleMode();
    }
    
    if (key == GLUT_KEY_ESCAPE) {
        exit(0);
    }
    
    if (gameState == STATE_MENU && key == ' ') {
        gameState = STATE_NEO_TOKYO;
        initializeNeoTokyo();
        mouseFirstMove = true;  // Reset mouse tracking when starting
    }
}

void keyboardUp(unsigned char key, int x, int y) {
    keys[key] = false;
}

void specialKeyboard(int key, int x, int y) {
    keys[key] = true;
}

void specialKeyboardUp(int key, int x, int y) {
    keys[key] = false;
}

void mouseMotion(int x, int y) {
    if (mouseFirstMove) {
        lastMouseX = x;
        lastMouseY = y;
        mouseFirstMove = false;
        return;
    }
    
    int dx = x - lastMouseX;
    int dy = y - lastMouseY;
    
    if (camera.mode == CAMERA_FIRST_PERSON) {
        // =========================================
        // FIRST-PERSON: Mouse controls player look direction
        // =========================================
        float sensitivity = 0.2f;  // Sensitivity for first-person look
        
        // Update player's horizontal look direction (yaw)
        // Move mouse right = look right (normal)
        player.yaw += dx * sensitivity;
        
        // Update player's vertical look direction (pitch)
        // Move mouse down = look down (normal)
        player.pitch -= dy * sensitivity;
        
        // Wrap yaw to 0-360 range
        while (player.yaw > 360.0f) player.yaw -= 360.0f;
        while (player.yaw < 0.0f) player.yaw += 360.0f;
        
        // Also update targetYaw to match (prevents snapping when switching modes)
        player.targetYaw = player.yaw;
        
        // Clamp pitch to prevent looking too far up/down
        if (player.pitch > 89.0f) player.pitch = 89.0f;
        if (player.pitch < -89.0f) player.pitch = -89.0f;
    } else {
        // =========================================
        // THIRD-PERSON: Mouse controls camera orbit around player
        // =========================================
        float sensitivity = 0.3f;  // Camera orbit sensitivity
        
        // Update camera yaw (horizontal orbit around player)
        // Move mouse right = camera orbits left (inverted)
        camera.cameraYaw -= dx * sensitivity;
        
        // Update camera pitch (vertical angle)
        // Move mouse up = camera looks more upward (inverted)
        camera.cameraPitch -= dy * sensitivity * 0.5f;
        
        // Wrap camera yaw to 0-360 range
        while (camera.cameraYaw > 360.0f) camera.cameraYaw -= 360.0f;
        while (camera.cameraYaw < 0.0f) camera.cameraYaw += 360.0f;
        
        // Clamp camera pitch to prevent flipping
        if (camera.cameraPitch > 60.0f) camera.cameraPitch = 60.0f;
        if (camera.cameraPitch < -10.0f) camera.cameraPitch = -10.0f;
    }
    
    lastMouseX = x;
    lastMouseY = y;
}

void mouseButton(int button, int state, int x, int y) {
    mouseButtons[button] = (state == GLUT_DOWN);
    mouseX = x;
    mouseY = y;
}

void timer(int value) {
    updateGame(0.016f); // ~60 FPS
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

// =============================================================================
// Main Function
// =============================================================================
int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Chrono Heist");
    
    // Initialize OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glClearColor(0.15f, 0.15f, 0.2f, 1.0f);  // Slightly brighter background
    
    // Enable material properties for better lighting
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    
    // Set default material properties
    float matSpecular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float matShininess[1] = {50.0f};
    glMaterialfv(GL_FRONT, GL_SPECULAR, matSpecular);
    glMaterialfv(GL_FRONT, GL_SHININESS, matShininess);

    if (!initializeTextureAssets()) {
        printf("Failed to initialize texture assets.\n");
    }
    if (!loadMainCharacterModel()) {
        printf("Using fallback capsule player.\n");
    }
    
    // Load prop models (scarab, camera, console)
    loadPropModels();
    
    // Initialize input
    memset(keys, false, sizeof(keys));
    memset(mouseButtons, false, sizeof(mouseButtons));
    
    // Initialize game
    gameState = STATE_MENU;
    
    // Set up GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialKeyboard);
    glutSpecialUpFunc(specialKeyboardUp);
    glutMotionFunc(mouseMotion);
    glutPassiveMotionFunc(mouseMotion);
    glutMouseFunc(mouseButton);
    glutTimerFunc(16, timer, 0);
    
    // Enter main loop
    glutMainLoop();
    
    return 0;
}

