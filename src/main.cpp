#include <iostream>
#include <sstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <string>
#include "ShaderProgram.h"
#include "SoftBody.h"
#include "TetoMeshData.h"

// Window and camera settings
int gWindowWidth = 1024, gWindowHeight = 768;
GLFWwindow* gWindow = NULL;
bool gWireframe = false;
bool restore = false;
glm::mat4 model(1.0), view(1.0), projection(1.0);

struct Camera {
    // 初期カメラ位置
    glm::vec3 initialCameraPos = glm::vec3(0.0f, 3.0f, 20.0f);
    glm::vec3 cameraPos = initialCameraPos;  // 現在のカメラ位置
    glm::vec3 cameraTarget = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 cameraDirection;
    glm::vec3 cameraRight;
    glm::vec3 cameraUp;
    float MOUSE_SENSITIVITY = 0.2f;
    float gRadius = 40.0f;  // 初期カメラ距離に合わせる
    float gYaw = 0.0f;      // 初期値
    float gPitch = 0.0f;    // 初期値
    float gFOV = 45.0f;
    const double ZOOM_SENSITIVITY = -1.0;

    // 初期化時に計算
    void initialize() {
        // 初期カメラ位置に基づいてgRadius、gYaw、gPitchを計算
        gRadius = glm::length(initialCameraPos - cameraTarget);

        float horizontalDistance = glm::length(
            glm::vec2(initialCameraPos.x - cameraTarget.x,
                      initialCameraPos.z - cameraTarget.z));

        gPitch = glm::degrees(atan2(initialCameraPos.y - cameraTarget.y,
                                    horizontalDistance));
        gYaw = glm::degrees(atan2(initialCameraPos.x - cameraTarget.x,
                                  initialCameraPos.z - cameraTarget.z));

        updateVectors();
    }

    void updateVectors() {
        cameraDirection = glm::normalize(cameraPos - cameraTarget);
        cameraRight = glm::normalize(glm::cross(up, cameraDirection));
        cameraUp = glm::normalize(glm::cross(cameraDirection, cameraRight));
    }

    void updatePosition() {
        float yaw = glm::radians(gYaw);
        float pitch = glm::radians(gPitch);
        pitch = glm::clamp(pitch, -glm::pi<float>() / 2.0f + 0.1f, glm::pi<float>() / 2.0f - 0.1f);
        gRadius = glm::clamp(gRadius, 2.0f, 80.0f);

        cameraPos.x = cameraTarget.x + gRadius * cosf(pitch) * sinf(yaw);
        cameraPos.y = cameraTarget.y + gRadius * sinf(pitch);
        cameraPos.z = cameraTarget.z + gRadius * cosf(pitch) * cosf(yaw);

        updateVectors();
    }

    // 新しいカメラ位置を設定し、その値に基づいてパラメータを更新
    void setCameraPosition(const glm::vec3& newPos) {
        initialCameraPos = newPos;
        cameraPos = newPos;
        initialize();
    }
}OrbitCam;

// Raycast variables
glm::vec3 hit_position;
bool isDragging = false;

// Raycast class
class RayCast {
public:
    struct Ray {
        glm::vec3 origin;
        glm::vec3 direction;
    };

    struct RayHit {
        bool hit;
        float distance;
        glm::vec3 position;
        SoftBody* hitObject;
    };

    // Generate ray from screen coordinates
    static Ray screenToRay(float screenX, float screenY,
                           const glm::mat4& view,
                           const glm::mat4& projection,
                           const glm::vec4& viewport) {
        // Convert screen coordinates to normalized device coordinates (NDC)
        float ndcX = (2.0f * screenX) / viewport.z - 1.0f;
        float ndcY = 1.0f - (2.0f * screenY) / viewport.w;  // Y-coordinate is inverted

        // Create near and far points in clip space
        glm::vec4 nearPoint = glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
        glm::vec4 farPoint = glm::vec4(ndcX, ndcY, 1.0f, 1.0f);

        // Calculate inverse view-projection matrix
        glm::mat4 invVP = glm::inverse(projection * view);

        // Transform to world space
        glm::vec4 worldNear = invVP * nearPoint;
        glm::vec4 worldFar = invVP * farPoint;

        // Divide by w component
        worldNear /= worldNear.w;
        worldFar /= worldFar.w;

        Ray ray;
        ray.origin = glm::vec3(worldNear);
        ray.direction = glm::normalize(glm::vec3(worldFar - worldNear));

        return ray;
    }

    // Ray-mesh intersection test
    static RayHit intersectMesh(const Ray& ray, SoftBody& mesh) {
        RayHit result = { false, std::numeric_limits<float>::max(), glm::vec3(0), nullptr };

        const auto& positions = mesh.getPositions();
        const auto& surfaceTriIds = mesh.getMeshData().tetSurfaceTriIds;

        float t, u, v;
        for (size_t i = 0; i < surfaceTriIds.size(); i += 3) {
            int idx1 = surfaceTriIds[i];
            int idx2 = surfaceTriIds[i + 1];
            int idx3 = surfaceTriIds[i + 2];

            // Vertex positions in model space
            glm::vec3 v1(positions[idx1 * 3], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2]);
            glm::vec3 v2(positions[idx2 * 3], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2]);
            glm::vec3 v3(positions[idx3 * 3], positions[idx3 * 3 + 1], positions[idx3 * 3 + 2]);

            if (rayTriangleIntersect(ray.origin, ray.direction, v1, v2, v3, t, u, v)) {
                if (t < result.distance) {
                    result.hit = true;
                    result.distance = t;
                    result.position = ray.origin + ray.direction * t;
                    result.hitObject = &mesh;
                }
            }
        }

        return result;
    }

private:
    // Ray-triangle intersection test (Möller–Trumbore algorithm)
    static bool rayTriangleIntersect(
        const glm::vec3& rayOrigin,
        const glm::vec3& rayDir,
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2,
        float& t,
        float& u,
        float& v) {

        const float EPSILON = 0.0000001f;
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 h = glm::cross(rayDir, edge2);
        float a = glm::dot(edge1, h);

        if (a > -EPSILON && a < EPSILON) return false;

        float f = 1.0f / a;
        glm::vec3 s = rayOrigin - v0;
        u = f * glm::dot(s, h);

        if (u < 0.0f || u > 1.0f) return false;

        glm::vec3 q = glm::cross(s, edge1);
        v = f * glm::dot(rayDir, q);

        if (v < 0.0f || u + v > 1.0f) return false;

        t = f * glm::dot(edge2, q);
        return t > EPSILON;
    }
};

// Grabber class - for grabbing and moving objects
class Grabber {
public:
    Grabber() : physicsObject(nullptr), grabDistance(0.0f), prevPosition(0.0f), velocity(0.0f), time(0.0f) {}

    void setPhysicsObject(SoftBody* object) {
        physicsObject = object;
    }

    void startGrab(float screenX, float screenY) {
        if (!physicsObject) return;

        // Generate ray in world space
        RayCast::Ray worldRay = RayCast::screenToRay(screenX, screenY, view, projection,
                                                     glm::vec4(0, 0, gWindowWidth, gWindowHeight));

        // Create model matrix
        glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), OrbitCam.cameraTarget);
        glm::mat4 invModelMatrix = glm::inverse(modelMatrix);

        // Transform ray to model space
        RayCast::Ray localRay;
        localRay.origin = glm::vec3(invModelMatrix * glm::vec4(worldRay.origin, 1.0f));
        localRay.direction = glm::normalize(glm::vec3(invModelMatrix * glm::vec4(worldRay.direction, 0.0f)));

        // Intersection test in model space
        RayCast::RayHit hit = RayCast::intersectMesh(localRay, *physicsObject);

        if (hit.hit) {
            // Transform hit position back to world space
            glm::vec4 worldHitPos = modelMatrix * glm::vec4(hit.position, 1.0f);
            hit_position = glm::vec3(worldHitPos);

            grabDistance = glm::length(hit_position - worldRay.origin);
            prevPosition = hit_position;
            velocity = glm::vec3(0.0f);
            time = 0.0f;

            physicsObject->startGrab(hit_position);
            isDragging = true;
        }
    }

    void moveGrab(float screenX, float screenY, float deltaTime) {
        if (!physicsObject || !isDragging) return;

        RayCast::Ray worldRay = RayCast::screenToRay(screenX, screenY, view, projection,
                                                     glm::vec4(0, 0, gWindowWidth, gWindowHeight));

        // Calculate new position using grabDistance (in world space)
        glm::vec3 newPosition = worldRay.origin + worldRay.direction * grabDistance;

        if (time > 0.0f) {
            velocity = (newPosition - prevPosition) / time;
        }

        hit_position = newPosition;
        physicsObject->moveGrabbed(newPosition, velocity);
        prevPosition = newPosition;
        time = deltaTime;
    }

    void endGrab() {
        if (physicsObject && isDragging) {
            physicsObject->endGrab(hit_position, velocity);
            isDragging = false;
        }
    }

    void update(float deltaTime) {
        time += deltaTime;
    }

private:
    SoftBody* physicsObject;
    float grabDistance;
    glm::vec3 prevPosition;
    glm::vec3 velocity;
    float time;
};

Grabber* gGrabber = nullptr;

// Function prototypes
bool initOpenGL();
void glfw_onKey(GLFWwindow* window, int key, int scancode, int action, int mode);
void glfw_OnFramebufferSize(GLFWwindow* window, int width, int height);
void glfw_onMouseMove(GLFWwindow* window, double posX, double posY);
void glfw_onMouseScroll(GLFWwindow* window, double deltaX, double deltaY);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void showFPS(GLFWwindow* window);

// Add bunny as a global variable to be accessible from the key handler
SoftBody* bunny = nullptr;

int main() {
    // Initialize OpenGL
    if (!initOpenGL()) {
        std::cerr << "GLFW initialization failed" << std::endl;
        return -1;
    }

    // Initialize Cam
    OrbitCam.initialize();

    // Load shaders
    ShaderProgram shaderProgram;
    shaderProgram.loadShaders("../../../shader/basic.vert",
                              "../../../shader/basic.frag");

    std::string liverPath = "../../../model/liver.obj";
    SoftBody::MeshData liver_mesh = TetoMeshData::ReadVetexAndFace(liverPath);
    std::string tetfilepath = "../../../model/liver_tetrahedral_mesh.txt";
    SoftBody::MeshData tetmesh = SoftBody::loadTetMesh(tetfilepath);

    // Initialize softbody with Neohookean compliance
    // 第3引数: エッジコンプライアンス
    // 第4引数: 体積コンプライアンス
    // 第5引数: Neohookeanコンプライアンス
    bunny = new SoftBody(tetmesh, liver_mesh, 0.0001f, 0.0001f, 10.0f);

    // Set material parameters (Young's modulus, Poisson's ratio)
    // 肝臓のようなやわらかい臓器の物性値を設定
    bunny->setMaterialParameters(2000.0f, 0.49f);

    // Physics parameters
    float dt = 1.0f / 60.0f;
    glm::vec3 gravity(0.0f, -9.8f, 0.0f);

    // Initialize grabber
    Grabber grabber;
    gGrabber = &grabber;
    gGrabber->setPhysicsObject(bunny);

    // Set mouse button callback
    glfwSetMouseButtonCallback(gWindow, mouse_button_callback);

    // Main loop
    while (!glfwWindowShouldClose(gWindow)) {
        showFPS(gWindow);
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update grabber
        gGrabber->update(dt);

        // Update camera
        OrbitCam.updatePosition();
        view = glm::lookAt(OrbitCam.cameraPos, OrbitCam.cameraTarget, OrbitCam.up);

        // Update model matrix
        model = glm::translate(glm::mat4(1.0f), OrbitCam.cameraTarget);
        bunny->setModelMatrix(model);

        // Update projection matrix
        projection = glm::perspective(glm::radians(OrbitCam.gFOV),
                                      float(gWindowWidth) / float(gWindowHeight), 0.1f, 100.0f);

        // Configure shader
        shaderProgram.use();
        shaderProgram.setUniform("model", model);
        shaderProgram.setUniform("lightPos", OrbitCam.cameraPos);
        shaderProgram.setUniform("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
        shaderProgram.setUniform("view", view);
        shaderProgram.setUniform("projection", projection);
        shaderProgram.setUniform("vertColor", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

        // Physics simulation
        int numSubsteps = 1;
        float stepDt = dt / float(numSubsteps);

        for (int i = 0; i < numSubsteps; i++) {
            bunny->preSolve(stepDt, gravity);
            bunny->solve(stepDt);
            bunny->postSolve(stepDt);
        }

        bunny->updateTetMeshes();
        bunny->updateVisMesh();
        bunny->draw_Vis(shaderProgram);
        bunny->drawTetMesh(shaderProgram);

        // Shape restoration if needed
        if (restore) {
            bunny->applyShapeRestoration(1.0);
            restore = false;
        }

        glfwSwapBuffers(gWindow);
    }

    glfwTerminate();
    return 0;
}

// Initialize OpenGL
bool initOpenGL() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    gWindow = glfwCreateWindow(gWindowWidth, gWindowHeight, "SoftBody Simulation", NULL, NULL);
    if (!gWindow) {
        return false;
    }

    glfwMakeContextCurrent(gWindow);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        return false;
    }

    // Set callbacks
    glfwSetKeyCallback(gWindow, glfw_onKey);
    glfwSetFramebufferSizeCallback(gWindow, glfw_OnFramebufferSize);
    glfwSetCursorPosCallback(gWindow, glfw_onMouseMove);
    glfwSetScrollCallback(gWindow, glfw_onMouseScroll);
    glfwSetInputMode(gWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // OpenGL settings
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glViewport(0, 0, gWindowWidth, gWindowHeight);
    glEnable(GL_DEPTH_TEST);

    return true;
}

// Mouse button callback
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        if (action == GLFW_PRESS) {
            gGrabber->startGrab(xpos, ypos);
        } else if (action == GLFW_RELEASE) {
            gGrabber->endGrab();
        }
    }
}

// Keyboard callback
void glfw_onKey(GLFWwindow* window, int key, int scancode, int action, int mode) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }

    if (key == GLFW_KEY_F1 && action == GLFW_PRESS) {
        gWireframe = !gWireframe;
        glPolygonMode(GL_FRONT_AND_BACK, gWireframe ? GL_LINE : GL_FILL);
    }

    if (key == GLFW_KEY_F2 && action == GLFW_PRESS) {
        restore = true;
    }

    // ここからは新しいキー操作
    // F3キー: エッジ拘束のみを使用
    if (key == GLFW_KEY_F3 && action == GLFW_PRESS) {
        bunny->setEdgeCompliance(1.0f);  // エッジ拘束を有効化
        bunny->setVolCompliance(0.0f);   // 体積拘束を無効化
        bunny->setNeoCompliance(0.0f);   // Neohookean拘束を無効化
        std::cout << "Edge constraints only" << std::endl;
    }

    // F4キー: 体積拘束のみを使用
    if (key == GLFW_KEY_F4 && action == GLFW_PRESS) {
        bunny->setEdgeCompliance(0.0f);   // エッジ拘束を無効化
        bunny->setVolCompliance(1.0f);    // 体積拘束を有効化
        bunny->setNeoCompliance(0.0f);    // Neohookean拘束を無効化
        std::cout << "Volume constraints only" << std::endl;
    }

    // F5キー: Neohookean拘束のみを使用
    if (key == GLFW_KEY_F5 && action == GLFW_PRESS) {
        bunny->setEdgeCompliance(0.0f);    // エッジ拘束を無効化
        bunny->setVolCompliance(0.0f);     // 体積拘束を無効化
        bunny->setNeoCompliance(1.0f);     // Neohookean拘束を有効化
        std::cout << "Neohookean constraints only" << std::endl;
    }

    // F6キー: すべての拘束を使用
    if (key == GLFW_KEY_F6 && action == GLFW_PRESS) {
        bunny->setEdgeCompliance(1.0f);    // エッジ拘束を有効化
        bunny->setVolCompliance(1.0f);     // 体積拘束を有効化
        bunny->setNeoCompliance(1.0f);     // Neohookean拘束を有効化
        std::cout << "All constraints enabled" << std::endl;
    }

    // 1キー: 柔らかい材料
    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        bunny->setMaterialParameters(10.0f, 0.45f);  // 柔らかい材料（ゴムのような）
        std::cout << "Soft material" << std::endl;
    }

    // 2キー: 中程度の硬さ
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        bunny->setMaterialParameters(1000.0f, 0.45f);  // 中程度の硬さ
        std::cout << "Medium material" << std::endl;
    }

    // 3キー: 硬い材料
    if (key == GLFW_KEY_3 && action == GLFW_PRESS) {
        bunny->setMaterialParameters(10000000.0f, 0.45f);  // 硬い材料（プラスチックのような）
        std::cout << "Hard material" << std::endl;
    }
}

// Framebuffer resize callback
void glfw_OnFramebufferSize(GLFWwindow* window, int width, int height) {
    gWindowWidth = width;
    gWindowHeight = height;
    glViewport(0, 0, gWindowWidth, gWindowHeight);
}

// Mouse movement callback
void glfw_onMouseMove(GLFWwindow* window, double posX, double posY) {
    static glm::vec2 lastMousePos = glm::vec2(0, 0);

    // Handle dragging
    if (isDragging && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (gGrabber) {
            gGrabber->moveGrab(posX, posY, 1.0f/60.0f);
        }
    }
    // Handle camera rotation
    else if (!isDragging && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        OrbitCam.gYaw -= ((float)posX - lastMousePos.x) * OrbitCam.MOUSE_SENSITIVITY;
        OrbitCam.gPitch += ((float)posY - lastMousePos.y) * OrbitCam.MOUSE_SENSITIVITY;
    }

    lastMousePos.x = (float)posX;
    lastMousePos.y = (float)posY;
}

// Mouse scroll callback
void glfw_onMouseScroll(GLFWwindow* window, double deltaX, double deltaY) {
    double fov = OrbitCam.gFOV + deltaY * OrbitCam.ZOOM_SENSITIVITY;
    OrbitCam.gFOV = glm::clamp(fov, 1.0, 120.0);
}

// Display FPS
void showFPS(GLFWwindow* window) {
    static double previousSeconds = 0.0;
    static int frameCount = 0;
    double currentSeconds = glfwGetTime();
    double elapsedSeconds = currentSeconds - previousSeconds;

    // Update every 0.25 seconds
    if (elapsedSeconds > 0.25) {
        previousSeconds = currentSeconds;
        double fps = (double)frameCount / elapsedSeconds;
        double msPerFrame = 1000.0 / fps;

        // Update window title
        std::ostringstream outs;
        outs.precision(3);
        outs << std::fixed
             << "SoftBody Simulation" << "    "
             << "FPS: " << fps << "    "
             << "Frame Time: " << msPerFrame << " (ms)";
        glfwSetWindowTitle(window, outs.str().c_str());

        frameCount = 0;
    }

    frameCount++;
}
