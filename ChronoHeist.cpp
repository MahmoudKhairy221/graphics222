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
const float PLAYER_JUMP_FORCE = 0.5f;
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
const float THIRD_PERSON_DISTANCE = 5.0f;
const float THIRD_PERSON_HEIGHT = 2.0f;

const char* TEXTURE_DIR = "textures/";
const char* MODERN_TOKYO_BASE = "textures/modern_tokyo/basecolor.png";
const char* ANCIENT_EGYPT_BASE = "textures/ancient_egypt/basecolor.jpg";
const char* CHARACTER_SCENE_PATH = "main_character/scene.gltf";

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
class Player {
public:
    Vector3f position;
    Vector3f velocity;
    float yaw;          // Horizontal rotation
    float pitch;        // Vertical rotation
    bool onGround;
    int health;
    int score;
    int crystalsCollected;
    int scarabsCollected;

    Player() {
        position = Vector3f(0, 2, 0);
        velocity = Vector3f(0, 0, 0);
        yaw = 0.0f;  // Facing forward (positive Z direction)
        pitch = 0.0f;  // Look straight ahead initially
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

        // Ground collision
        if (position.y < GROUND_Y + PLAYER_HEIGHT / 2) {
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
    }

    void jump() {
        if (onGround) {
            velocity.y = PLAYER_JUMP_FORCE;
            onGround = false;
        }
    }

    void move(Vector3f direction) {
        // Movement is relative to where the player is facing
        // player.yaw is updated by mouse movement, so player rotates with camera
        // Forward is positive Z (standard OpenGL convention)
        // Calculate forward and right vectors based on current yaw (where player is facing)
        Vector3f forward = Vector3f(sin(DEG2RAD(yaw)), 0, cos(DEG2RAD(yaw)));
        Vector3f right = Vector3f(cos(DEG2RAD(yaw)), 0, -sin(DEG2RAD(yaw)));
        
        // Normalize direction for consistent speed
        if (direction.lengthSquared() > 0.0001f) {
            direction = direction.unit();
        }
        
        // Apply movement relative to player facing direction:
        // direction.z = forward/backward (W/S keys) - moves in direction player is facing
        // direction.x = left/right (A/D keys) - strafes left/right relative to facing direction
        Vector3f moveDir = (forward * direction.z + right * direction.x) * PLAYER_SPEED;
        velocity.x = moveDir.x;
        velocity.z = moveDir.z;
    }
};

// =============================================================================
// Camera Class
// =============================================================================
class Camera {
public:
    CameraMode mode;
    Vector3f eye;
    Vector3f center;
    Vector3f up;
    float distance;
    float angleX;
    float angleY;

    Camera() {
        mode = CAMERA_THIRD_PERSON;  // Start in third-person for better overview
        eye = Vector3f(0, 8, 10);
        center = Vector3f(0, 2, 0);
        up = Vector3f(0, 1, 0);
        distance = THIRD_PERSON_DISTANCE;
        angleX = 0.0f;
        angleY = -15.0f;  // Look down slightly for better view
    }

    void update(Player& player) {
        if (mode == CAMERA_FIRST_PERSON) {
            // First-person: camera at player eye level
            eye = player.position + Vector3f(0, PLAYER_HEIGHT * 0.4f, 0);
            
            // Calculate look direction from player yaw and pitch
            float radYaw = DEG2RAD(player.yaw);
            float radPitch = DEG2RAD(player.pitch);
            
            // Forward direction based on yaw and pitch
            float forwardDist = 5.0f;  // Look distance
            center = eye + Vector3f(
                sin(radYaw) * cos(radPitch) * forwardDist,
                sin(radPitch) * forwardDist,
                cos(radYaw) * cos(radPitch) * forwardDist
            );
        } else {
            // Third-person: camera orbits around player
            // angleX controls horizontal orbit, angleY controls vertical angle
            float radYaw = DEG2RAD(player.yaw + angleX + 180);  // Behind player + orbit angle
            float radPitch = DEG2RAD(angleY);
            
            // Calculate camera offset
            float camDistance = distance;
            float camHeight = THIRD_PERSON_HEIGHT + sin(radPitch) * 2.0f;
            
            Vector3f offset = Vector3f(
                sin(radYaw) * cos(radPitch) * camDistance,
                camHeight,
                cos(radYaw) * cos(radPitch) * camDistance
            );
            
            eye = player.position + offset;
            
            // Look at player with slight offset upward
            center = player.position + Vector3f(0, PLAYER_HEIGHT * 0.5f, 0);
        }
    }

    void look() {
        gluLookAt(eye.x, eye.y, eye.z,
                  center.x, center.y, center.z,
                  up.x, up.y, up.z);
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
CharacterMeshData mainCharacterMesh;
bool textureSystemReady = false;

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

    textureSystemReady = true;
    return true;
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

bool loadMainCharacterModel() {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err;
    std::string warn;

    bool ok = loader.LoadASCIIFromFile(&model, &err, &warn, CHARACTER_SCENE_PATH);
    if (!warn.empty()) {
        printf("GLTF warning: %s\n", warn.c_str());
    }
    if (!ok) {
        printf("Failed to load character model: %s\n", err.c_str());
        return false;
    }

    if (model.meshes.empty()) {
        printf("Character GLTF has no meshes.\n");
        return false;
    }

    const tinygltf::Mesh& mesh = model.meshes[0];
    if (mesh.primitives.empty()) {
        printf("Character mesh has no primitives.\n");
        return false;
    }

    const tinygltf::Primitive& primitive = mesh.primitives[0];
    auto posIt = primitive.attributes.find("POSITION");
    if (posIt == primitive.attributes.end()) {
        printf("Character mesh missing POSITION data.\n");
        return false;
    }

    const tinygltf::Accessor& posAccessor = model.accessors[posIt->second];
    if (!readAccessorData(model, posAccessor, mainCharacterMesh.positions, 3)) {
        printf("Failed to read POSITION data.\n");
        return false;
    }

    auto normalIt = primitive.attributes.find("NORMAL");
    if (normalIt != primitive.attributes.end()) {
        readAccessorData(model, model.accessors[normalIt->second], mainCharacterMesh.normals, 3);
    }

    auto uvIt = primitive.attributes.find("TEXCOORD_0");
    if (uvIt != primitive.attributes.end()) {
        readAccessorData(model, model.accessors[uvIt->second], mainCharacterMesh.texcoords, 2);
    }

    if (primitive.indices >= 0) {
        const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
        readIndexData(model, indexAccessor, mainCharacterMesh.indices);
    } else {
        size_t vertexCount = posAccessor.count;
        mainCharacterMesh.indices.resize(vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            mainCharacterMesh.indices[i] = static_cast<unsigned int>(i);
        }
    }

    // Compute bounds
    if (!mainCharacterMesh.positions.empty()) {
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
    }

    // Load texture
    if (primitive.material >= 0 && primitive.material < model.materials.size()) {
        const auto& material = model.materials[primitive.material];
        if (material.pbrMetallicRoughness.baseColorTexture.index >= 0) {
            const tinygltf::Texture& tex = model.textures[material.pbrMetallicRoughness.baseColorTexture.index];
            if (tex.source >= 0 && tex.source < model.images.size()) {
                mainCharacterMesh.albedo = textureFromTinyImage(model.images[tex.source]);
            }
        }
    }

    if (!mainCharacterMesh.albedo.valid()) {
        mainCharacterMesh.albedo = fallbackTexture;
    }

    mainCharacterMesh.loaded = true;
    printf("Loaded character mesh with %zu vertices and %zu triangles.\n",
           mainCharacterMesh.positions.size() / 3,
           mainCharacterMesh.indices.size() / 3);
    return true;
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
    glPushMatrix();
    glTranslatef(pos.x, pos.y + bob, pos.z);
    glRotatef(rotation, 0, 1, 0);

    // Emissive material for scarab
    float emissive[4] = {0.8f, 0.6f, 0.2f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
    glColor3f(0.9f, 0.7f, 0.1f);

    // Scarab body (oval)
    glScalef(0.4f, 0.2f, 0.6f);
    drawSphere(1.0f, 16, 16);

    // Scarab head
    glPushMatrix();
    glTranslatef(0, 0, 0.7f);
    glScalef(0.3f, 0.3f, 0.3f);
    drawSphere(1.0f, 12, 12);
    glPopMatrix();

    // Scarab wings (carved inlay style)
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

void drawPlayer() {
    glPushMatrix();
    // Translate to player position first
    glTranslatef(player.position.x, player.position.y, player.position.z);
    // Rotate player to match camera direction (player.yaw is updated by mouse movement)
    // This makes the player rotate with the camera when you move the mouse
    glRotatef(player.yaw, 0, 1, 0);  // Rotate around Y-axis (vertical) by player.yaw degrees

    if (mainCharacterMesh.loaded) {
        // For GLTF model: scale, then center, then draw
        glScalef(mainCharacterMesh.scale, mainCharacterMesh.scale, mainCharacterMesh.scale);
        glTranslatef(-mainCharacterMesh.boundsCenter.x,
                     -mainCharacterMesh.boundsCenter.y,
                     -mainCharacterMesh.boundsCenter.z);
        drawLoadedCharacter();
    } else {
        // Fallback capsule player - rotation is already applied above
        glColor3f(0.2f, 0.4f, 0.8f);
        
        // Body
        glPushMatrix();
        glTranslatef(0, 0.5f, 0);
        drawCube(0.6f);
        glPopMatrix();

        // Head
        glPushMatrix();
        glTranslatef(0, 1.2f, 0);
        drawSphere(0.3f, 12, 12);
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
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(rotation, 0, 1, 0);

    glColor3f(0.3f, 0.3f, 0.3f);
    
    // Camera body
    glPushMatrix();
    glTranslatef(0, 0, 0);
    drawCube(0.4f);
    glPopMatrix();

    // Camera lens
    glPushMatrix();
    glTranslatef(0, 0, 0.25f);
    glColor3f(0.1f, 0.1f, 0.1f);
    drawSphere(0.15f, 12, 12);
    glPopMatrix();

    // Detection cone (visual)
    if (alarmActive) {
        glColor4f(1.0f, 0.0f, 0.0f, 0.2f);
        glBegin(GL_TRIANGLES);
        glVertex3f(0, 0, 0);
        float angle1 = -sweepAngle * 0.5f;
        float angle2 = sweepAngle * 0.5f;
        float dist = 8.0f;
        glVertex3f(sin(DEG2RAD(angle1)) * dist, -2.0f, cos(DEG2RAD(angle1)) * dist);
        glVertex3f(sin(DEG2RAD(angle2)) * dist, -2.0f, cos(DEG2RAD(angle2)) * dist);
        glEnd();
    }

    glPopMatrix();
}

void drawControlConsole(Vector3f pos, bool activated) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);

    glColor3f(0.1f, 0.3f, 0.5f);
    
    // Console base
    drawCube(1.5f);
    
    // Console screen
    glPushMatrix();
    glTranslatef(0, 0.8f, 0.76f);
    if (activated) {
        glColor3f(0.0f, 1.0f, 0.0f);
    } else {
        glColor3f(0.2f, 0.2f, 0.4f);
    }
    drawCube(1.0f);
    glPopMatrix();

    glPopMatrix();
}

void drawExitPortal(Vector3f pos, bool unlocked, float pulse) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);

    // Portal ring
    float scale = 1.0f + pulse * 0.3f;
    glScalef(scale, scale, scale);

    // Emissive material
    float emissive[4] = {0.2f + pulse * 0.3f, 0.4f + pulse * 0.4f, 0.8f + pulse * 0.2f, 1.0f};
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

    glColor3f(0.6f, 0.5f, 0.4f);
    
    // Body
    glPushMatrix();
    glTranslatef(0, 1.5f, 0);
    drawCube(1.0f);
    glPopMatrix();

    // Head
    glPushMatrix();
    glTranslatef(0, 2.5f, 0);
    drawSphere(0.4f, 16, 16);
    glPopMatrix();

    // Arms
    glPushMatrix();
    glTranslatef(-0.7f, 1.5f, 0);
    drawCylinder(0.15f, 0.15f, 1.0f, 12, 1);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.7f, 1.5f, 0);
    drawCylinder(0.15f, 0.15f, 1.0f, 12, 1);
    glPopMatrix();

    glPopMatrix();
}

void drawNeoTokyoEnvironment() {
    // Floor + walls texture
    float floorRepeat = 8.0f;
    useTexture(neoTokyoSurface);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glTexCoord2f(0, 0); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, 0); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MIN);
    glTexCoord2f(floorRepeat, floorRepeat); glVertex3f(WORLD_MAX, GROUND_Y, WORLD_MAX);
    glTexCoord2f(0, floorRepeat); glVertex3f(WORLD_MIN, GROUND_Y, WORLD_MAX);
    glEnd();

    float wallRepeat = 6.0f;
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

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Decorative wall panels
    for (int i = 0; i < 8; i++) {
        float x = WORLD_MIN + (i % 4) * 15.0f;
        float z = (i < 4) ? WORLD_MIN + 1.0f : WORLD_MAX - 1.0f;
        glPushMatrix();
        glTranslatef(x, 3.0f, z);
        glColor3f(0.25f, 0.3f, 0.35f);
        drawCube(2.0f);
        glPopMatrix();
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

    // Walls with hieroglyphic texture pattern
    float wallRepeat = 4.0f;
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

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Ancient columns
    for (int i = 0; i < 6; i++) {
        float angle = (i / 6.0f) * 2.0f * M_PI;
        float radius = 12.0f;
        glPushMatrix();
        glTranslatef(cos(angle) * radius, 0, sin(angle) * radius);
        glColor3f(0.6f, 0.55f, 0.5f);
        drawCylinder(0.8f, 0.8f, 8.0f, 16, 1);
        glPopMatrix();
    }

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

    // Draw obstacles
    // Pressure Plates
    static bool plateTriggered[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        float x = -10.0f + i * 10.0f;
        drawPressurePlate(Vector3f(x, 0.5f, -10), plateTriggered[i]);
    }

    // Moving Platforms
    float platformOffset = sin(gameTime * 2.0f) * 2.0f;
    drawMovingPlatform(Vector3f(0, 3, 0), platformOffset);

    // Guardian Statue
    float guardRotation = fmod(gameTime * 20.0f, 360.0f);
    drawGuardianStatue(Vector3f(15, 1, 15), guardRotation);

    // Exit Portal
    bool portalReady = player.scarabsCollected >= SCARABS_REQUIRED;
    float portalPulse = sin(gameTime * 3.0f) * 0.5f + 0.5f;
    drawExitPortal(Vector3f(-20, 2, -20), portalReady, portalPulse);
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
    
    player.position = Vector3f(0, 2, 0);
    player.velocity = Vector3f(0, 0, 0);
    player.yaw = 0.0f;
    player.pitch = 0.0f;  // Look straight ahead
    player.crystalsCollected = 0;
    consoleActivated = false;
    portalUnlocked = false;
    alarmActive = false;
    hitReactionActive = false;
    lighting.currentScene = 0;
    
    // Initialize camera properly
    camera.mode = CAMERA_THIRD_PERSON;
    camera.angleY = -15.0f;  // Slight downward angle
    camera.update(player);
    mouseFirstMove = true;  // Reset mouse tracking
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
    
    player.position = Vector3f(0, 2, 0);
    player.velocity = Vector3f(0, 0, 0);
    player.yaw = 0.0f;
    player.pitch = 0.0f;  // Look straight ahead
    player.scarabsCollected = 0;
    hitReactionActive = false;
    lighting.currentScene = 1;
    
    // Initialize camera properly
    camera.mode = CAMERA_THIRD_PERSON;
    camera.angleY = -15.0f;  // Slight downward angle
    camera.update(player);
    mouseFirstMove = true;  // Reset mouse tracking
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

void checkObstacles() {
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

        // Control Console interaction
        Vector3f consolePos(-20, 1, -20);
        if ((player.position - consolePos).lengthSquared() < 4.0f) {
            if (keys['E'] || keys['e']) {
                if (!consoleActivated) {
                    consoleActivated = true;
                    portalUnlocked = true;
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
        // Pressure Plate traps
        for (int i = 0; i < 3; i++) {
            float x = -10.0f + i * 10.0f;
            Vector3f platePos(x, 0.5f, -10);
            if ((player.position - platePos).lengthSquared() < 2.0f) {
                player.health -= 2;
                hitReactionActive = true;
                hitReactionTime = gameTime;
                playSFX("trap_trigger");
            }
        }

        // Moving Platform collision
        Vector3f platformPos(0, 3, 0);
        float platformOffset = sin(gameTime * 2.0f) * 2.0f;
        Vector3f platformWorldPos = platformPos + Vector3f(0, platformOffset, 0);
        if ((player.position - platformWorldPos).lengthSquared() < 4.0f && 
            player.position.y < platformWorldPos.y + 1.5f &&
            player.position.y > platformWorldPos.y - 1.5f) {
            player.position.y = platformWorldPos.y + 1.5f;
            player.onGround = true;
            player.velocity.y = 0;
        }

        // Guardian Statue collision
        Vector3f guardPos(15, 1, 15);
        if ((player.position - guardPos).lengthSquared() < 4.0f) {
            player.health -= 3;
            Vector3f knockback = (player.position - guardPos).unit() * 2.0f;
            player.velocity = player.velocity + knockback;
            hitReactionActive = true;
            hitReactionTime = gameTime;
            playSFX("guardian_hit");
        }

        // Exit Portal
        Vector3f portalPos(-20, 2, -20);
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

    // Clean up finished pickup animations
    for (size_t i = pickupAnimations.size(); i > 0; i--) {
        if (!pickupAnimations[i-1].active) {
            pickupAnimations.erase(pickupAnimations.begin() + i - 1);
        }
    }

    // Auto-switch to temple after 30 seconds (dummy test)
    if (gameState == STATE_NEO_TOKYO && gameTime >= 30.0f) {
        gameState = STATE_TEMPLE;
        initializeTemple();
        return;
    }

    // Update lighting
    lighting.update(deltaTime);

    // Update player movement - relative to where player is facing
    // The player rotates with the camera (player.yaw is updated by mouse movement)
    // W = forward (in direction player is facing), S = backward, A = left, D = right
    Vector3f moveDir(0, 0, 0);
    if (keys['W'] || keys['w'] || keys[GLUT_KEY_UP]) moveDir.z = 1.0f;   // Forward (relative to player.yaw)
    if (keys['S'] || keys['s'] || keys[GLUT_KEY_DOWN]) moveDir.z = -1.0f; // Backward (relative to player.yaw)
    if (keys['A'] || keys['a'] || keys[GLUT_KEY_LEFT]) moveDir.x = -1.0f;  // Left (relative to player.yaw)
    if (keys['D'] || keys['d'] || keys[GLUT_KEY_RIGHT]) moveDir.x = 1.0f;  // Right (relative to player.yaw)
    
    if (moveDir.lengthSquared() > 0.0001f) {
        // Player::move() converts input direction to world space based on player.yaw
        // Since player.yaw rotates with camera, movement is relative to where player is facing
        player.move(moveDir);
    }
    
    if (keys[' ']) {
        player.jump();
    }

    player.update();

    // Update camera
    camera.update(player);

    // Check collisions and interactions
    checkCollectibles();
    checkObstacles();

    // Check win/lose conditions
    if (player.health <= 0) {
        gameState = STATE_LOSE;
        playSFX("game_over");
    }

    // Check fatal pit (Temple)
    if (gameState == STATE_TEMPLE && player.position.y < -5.0f) {
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
    
    if (key == 'c' || key == 'C') {
        camera.toggleMode();
    }
    
    if (key == GLUT_KEY_ESCAPE) {
        exit(0);
    }
    
    if (gameState == STATE_MENU && key == ' ') {
        gameState = STATE_NEO_TOKYO;
        initializeNeoTokyo();
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
    
    // Smooth mouse sensitivity - reduced for easier control
    float sensitivity = 0.1f;  // Camera rotation sensitivity
    
    // Check if mouse button is held down (works in both first-person and third-person modes)
    bool mouseHeld = mouseButtons[GLUT_LEFT_BUTTON] || mouseButtons[GLUT_RIGHT_BUTTON] || mouseButtons[GLUT_MIDDLE_BUTTON];
    
    if (mouseHeld) {
        // Mouse button held: Rotate the PLAYER (works in both first-person and third-person)
        player.yaw += dx * sensitivity;
        
        // Wrap yaw to keep it in reasonable range (prevents overflow)
        while (player.yaw > 360.0f) player.yaw -= 360.0f;
        while (player.yaw < 0.0f) player.yaw += 360.0f;
        
        // Also update pitch when rotating player
        player.pitch += dy * sensitivity;
        
        // Clamp pitch to prevent flipping
        if (player.pitch > 89.0f) player.pitch = 89.0f;
        if (player.pitch < -89.0f) player.pitch = -89.0f;
        
        // In third-person, also update camera angle to follow player rotation
        if (camera.mode == CAMERA_THIRD_PERSON) {
            // Camera follows player rotation, so angleX stays relative to player
            // No need to update angleX here since it's relative to player.yaw
        }
    } else {
        // No mouse button: Rotate only the CAMERA (not the player)
        // This works differently in first-person vs third-person
        if (camera.mode == CAMERA_THIRD_PERSON) {
            // In third-person, adjust camera angle around player (orbit camera)
            camera.angleX += dx * sensitivity;
            // Wrap angleX to keep it in reasonable range
            while (camera.angleX > 360.0f) camera.angleX -= 360.0f;
            while (camera.angleX < 0.0f) camera.angleX += 360.0f;
            
            camera.angleY += dy * sensitivity * 0.3f;
            if (camera.angleY > 45.0f) camera.angleY = 45.0f;
            if (camera.angleY < -45.0f) camera.angleY = -45.0f;
        } else {
            // In first-person, only update pitch for camera look direction
            // Don't update player.yaw - this keeps player facing same direction
            player.pitch += dy * sensitivity;
            if (player.pitch > 89.0f) player.pitch = 89.0f;
            if (player.pitch < -89.0f) player.pitch = -89.0f;
            // Horizontal camera rotation follows player.yaw in first-person, so no change needed
        }
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

