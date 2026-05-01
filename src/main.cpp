#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <cstdlib>

#include "Shader.h"
#include "ShaderSources.h"
#include "Camera.h"
#include "PointCloud.h"
#include "PlyLoader.h"

namespace fs = std::filesystem;

struct AppState
{
    glm::vec3 backgroundColor{0.0f, 0.0f, 0.0f};
    bool showMesh = false;
    bool showPoints = true;
    bool play = false;
    bool mouseLook = false;

    glm::vec3 pointColor{0.82f, 0.82f, 0.82f};
    glm::vec3 meshColorDark{0.58f, 0.58f, 0.56f};
    glm::vec3 meshColorLight{0.42f, 0.42f, 0.40f};

    float pointSize = 1.0f;
    float renderDistance = 8.0f;
    float buildDegrees = 360.0f;
    float meshOpacity = 1.0f;
    float animationSpeedDegPerSec = 45.0f;

    int maxPoints = 0;
};

struct GpuBuffer
{
    unsigned int vao = 0;
    unsigned int vbo = 0;
    unsigned int ebo = 0;
    int vertexCount = 0;
    int indexCount = 0;

    void destroy()
    {
        if (ebo)
            glDeleteBuffers(1, &ebo);
        if (vbo)
            glDeleteBuffers(1, &vbo);
        if (vao)
            glDeleteVertexArrays(1, &vao);
        vao = vbo = ebo = 0;
        vertexCount = indexCount = 0;
    }
};

static Camera gCamera;
static AppState gState;
static bool gFirstMouse = true;
static double gLastMouseX = 0.0;
static double gLastMouseY = 0.0;

bool isBackgroundLight()
{
    float brightness =
        0.2126f * gState.backgroundColor.r +
        0.7152f * gState.backgroundColor.g +
        0.0722f * gState.backgroundColor.b;
    return brightness > 0.55f;
}

fs::path findProjectRoot(const fs::path &start)
{
    fs::path p = fs::absolute(start);

    for (int i = 0; i < 8; ++i)
    {
        if (fs::exists(p / "scan_output" / "room_scan.csv"))
            return p;

        if (p.has_parent_path())
            p = p.parent_path();
        else
            break;
    }

    return fs::absolute(start);
}

fs::path parseRootArg(int argc, char **argv)
{
    fs::path root = fs::current_path();

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];

        if (a == "--root" && i + 1 < argc)
        {
            root = fs::path(argv[++i]);
        }
        else if (a == "--max-points" && i + 1 < argc)
        {
            gState.maxPoints = std::stoi(argv[++i]);
        }
    }

    return findProjectRoot(root);
}

fs::path findMeshPath(const fs::path &root)
{
    fs::path meshPath = root / "tsdf_output" / "tsdf_room_mesh.ply";
    return fs::exists(meshPath) ? meshPath : fs::path{};
}

std::string quotePath(const fs::path &p)
{
    return std::string("\"") + p.string() + std::string("\"");
}

std::string displayPath(const fs::path &p)
{
    return p.lexically_normal().generic_string();
}

// mainly to avoid loading the mesh everysingle time while i debug
bool askYesNo(const std::string &prompt, bool defaultValue = false)
{
    std::cout << prompt;
    std::string answer;
    std::getline(std::cin, answer);

    if (answer.empty())
        return defaultValue;

    char c = static_cast<char>(std::tolower(static_cast<unsigned char>(answer[0])));
    return c == 'y';
}

std::string askMeshPreset()
{
    std::cout << "\nChoose TSDF mesh quality preset:\n";
    std::cout << "  1) ultra    best quality, slowest\n";
    std::cout << "  2) high     \n";
    std::cout << "  3) balanced \n";
    std::cout << "  4) fast     \n";
    std::cout << "Preset [ultra/high/balanced/fast] default=ultra: ";

    std::string answer;
    std::getline(std::cin, answer);

    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });

    if (answer.empty() || answer == "1" || answer == "u" || answer == "ultra")
        return "ultra";

    if (answer == "2" || answer == "h" || answer == "high")
        return "high";

    if (answer == "3" || answer == "b" || answer == "balanced")
        return "balanced";

    if (answer == "4" || answer == "f" || answer == "fast")
        return "fast";

    std::cout << "Unknown preset '" << answer << "'. Using ultra.\n";
    return "ultra";
}

bool generateTsdfMeshIfRequested(const fs::path &root)
{
    fs::path script = root / "tsdf_range_fusion" / "tsdf_fuse_organized.py";

    std::cout << "\nMesh startup option\n";
    std::cout << "  Expected TSDF script: " << displayPath(script) << "\n";
    std::cout << "  Expected output:      " << displayPath(root / "tsdf_output" / "tsdf_room_mesh.ply") << "\n";

    bool wantsMesh = askYesNo("Generate/update TSDF mesh now? [Y/N]: ", false);

    if (!wantsMesh)
    {
        std::cout << "Skipping mesh generation.\n\n";
        return false;
    }

    std::string detailPreset = askMeshPreset();

    if (!fs::exists(script))
    {
        std::cout << "Could not find TSDF script:\n  " << displayPath(script) << "\n";
        std::cout << "Skipping mesh generation. Put tsdf_fuse_organized.py inside:\n  "
                  << displayPath(root / "tsdf_range_fusion") << "\n\n";
        return false;
    }

    //  i have these in root
    // scan_output/room_scan.csv
    // tsdf_output/tsdf_room_mesh.ply
#if defined(_WIN32)
    std::string command =
        "cd /d " + quotePath(root) +
        " && python " + quotePath(script) +
        " --detail-preset " + detailPreset +
        " --fill-holes-size 0.35";
#else
    std::string command =
        "cd " + quotePath(root) +
        " && python3 " + quotePath(script) +
        " --detail-preset " + detailPreset +
        " --fill-holes-size 0.35";
#endif

    std::cout << "\nSelected mesh preset: " << detailPreset << "\n";
    std::cout << "Running mesh command from root: " << displayPath(root) << "\n";
    std::cout << command << "\n\n";
    int result = std::system(command.c_str());

    if (result != 0)
    {
        std::cout << "\nMesh generation command failed with code: " << result << "\n";
        std::cout << "The viewer will still open and use any existing mesh if one is present.\n\n";
        return false;
    }

    std::cout << "\nMesh generation finished.\n\n";
    return true;
}

void glfwErrorCallback(int code, const char *msg)
{
    std::cerr << "GLFW error " << code << ": " << msg << std::endl;
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (action == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse)
        {
            gState.mouseLook = true;
            gFirstMouse = true;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
        else if (action == GLFW_RELEASE)
        {
            gState.mouseLook = false;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    }
}

void cursorCallback(GLFWwindow *window, double xpos, double ypos)
{
    if (!gState.mouseLook)
        return;

    if (gFirstMouse)
    {
        gLastMouseX = xpos;
        gLastMouseY = ypos;
        gFirstMouse = false;
    }

    double dx = xpos - gLastMouseX;
    double dy = gLastMouseY - ypos;

    gLastMouseX = xpos;
    gLastMouseY = ypos;

    gCamera.addYawPitch(static_cast<float>(dx), static_cast<float>(dy));
}

void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    if (ImGui::GetIO().WantCaptureMouse)
        return;

    if (yoffset > 0)
        gCamera.speed *= 1.15f;
    if (yoffset < 0)
        gCamera.speed /= 1.15f;
    gCamera.speed = std::max(gCamera.speed, 0.01f);
}

void processInput(GLFWwindow *window, float dt, const glm::vec3 &center, float sceneRadius)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        gState.mouseLook = false;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    if (!ImGui::GetIO().WantCaptureKeyboard)
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            gCamera.moveForward(dt);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            gCamera.moveBackward(dt);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            gCamera.moveLeft(dt);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            gCamera.moveRight(dt);
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
            gCamera.moveDown(dt);
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
            gCamera.moveUp(dt);

        static bool bLast = false;
        bool bNow = glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS;
        if (bNow && !bLast)
        {
            gState.backgroundColor = isBackgroundLight()
                                         ? glm::vec3(0.0f, 0.0f, 0.0f)
                                         : glm::vec3(1.0f, 1.0f, 1.0f);
        }
        bLast = bNow;

        static bool mLast = false;
        bool mNow = glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS;
        if (mNow && !mLast)
            gState.showMesh = !gState.showMesh;
        mLast = mNow;

        static bool nLast = false;
        bool nNow = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
        if (nNow && !nLast)
            gState.showPoints = !gState.showPoints;
        nLast = nNow;

        static bool spaceLast = false;
        bool spaceNow = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        if (spaceNow && !spaceLast)
        {
            bool startingPlayback = !gState.play;
            if (startingPlayback && gState.buildDegrees >= 359.9f)
            {
                gState.buildDegrees = 0.0f;
            }
            if (startingPlayback)
            {
                gState.showPoints = true;
            }
            gState.play = !gState.play;
        }
        spaceLast = spaceNow;

        static bool rLast = false;
        bool rNow = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        if (rNow && !rLast)
            gCamera.reset(center, sceneRadius);
        rLast = rNow;
    }
}

GpuBuffer uploadPointCloud(const PointCloudData &cloud)
{
    GpuBuffer gpu;
    gpu.vertexCount = static_cast<int>(cloud.vertices.size());

    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);

    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        cloud.vertices.size() * sizeof(PointVertex),
        cloud.vertices.data(),
        GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PointVertex), (void *)offsetof(PointVertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(PointVertex), (void *)offsetof(PointVertex, radius));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(PointVertex), (void *)offsetof(PointVertex, progress));

    glBindVertexArray(0);
    return gpu;
}

struct MeshVertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

GpuBuffer uploadMesh(const MeshData &mesh)
{
    GpuBuffer gpu;
    gpu.vertexCount = static_cast<int>(mesh.vertices.size());
    gpu.indexCount = static_cast<int>(mesh.indices.size());

    std::vector<MeshVertex> verts;
    verts.reserve(mesh.vertices.size());

    for (size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        glm::vec3 n = i < mesh.normals.size() ? mesh.normals[i] : glm::vec3(0.0f, 1.0f, 0.0f);
        verts.push_back({mesh.vertices[i], n});
    }

    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ebo);

    glBindVertexArray(gpu.vao);

    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(MeshVertex), verts.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void *)offsetof(MeshVertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void *)offsetof(MeshVertex, normal));

    glBindVertexArray(0);
    return gpu;
}

void drawUi(const fs::path &csvPath, const fs::path &meshPath, const PointCloudData &cloud)
{
    ImGuiIO &io = ImGui::GetIO();

    const float panelWidth = 380.0f;
    const float panelHeight = 340.0f;

    // maybe make it moveable later??
    ImVec2 pos(18.0f, 18.0f);

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_Always);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoMove;

    ImGui::Begin("LiDAR Viewer", nullptr, flags);

    ImGui::Text("Points: %d", static_cast<int>(cloud.vertices.size()));
    ImGui::Text("CSV: %s", csvPath.filename().string().c_str());

    if (!meshPath.empty())
        ImGui::Text("Mesh: %s", meshPath.filename().string().c_str());
    else
        ImGui::Text("Mesh: none found");

    ImGui::Separator();

    ImGui::ColorEdit3("Background color", &gState.backgroundColor.x);
    ImGui::ColorEdit3("Point color", &gState.pointColor.x);

    ImGui::SliderFloat("Point size", &gState.pointSize, 1.0f, 18.0f, "%.1f");
    ImGui::SliderFloat("Render distance", &gState.renderDistance, 0.0f, cloud.maxRadius, "%.2f m");

    ImGui::SliderFloat("Build progress", &gState.buildDegrees, 0.0f, 360.0f, "%.0f deg");

    if (ImGui::Button(gState.play ? "Pause build animation" : "Play build animation"))
    {
        bool startingPlayback = !gState.play;
        if (startingPlayback && gState.buildDegrees >= 359.9f)
        {
            gState.buildDegrees = 0.0f;
        }
        if (startingPlayback)
        {
            gState.showPoints = true;
        }
        gState.play = !gState.play;
    }

    ImGui::SameLine();

    if (ImGui::Button("Reset build"))
    {
        gState.buildDegrees = 0.0f;
        gState.play = false;
    }

    ImGui::Separator();

    ImGui::BeginDisabled(meshPath.empty());
    ImGui::Checkbox("Show mesh underneath (M)", &gState.showMesh);
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::Checkbox("Toggle Points (N)", &gState.showPoints);

    ImGui::SliderFloat("Mesh opacity", &gState.meshOpacity, 0.0f, 1.0f, "%0.2f");

    ImGui::Separator();

    ImGui::SliderFloat("Camera speed", &gCamera.speed, 0.05f, std::max(cloud.sceneRadius * 2.0f, 3.0f), "%.2f");
    ImGui::TextWrapped("WASD move | Q/E up/down | Right mouse look | M mesh | N points | Space play | R reset | Esc release mouse");

    ImGui::End();
}

int main(int argc, char **argv)
{
    fs::path root = parseRootArg(argc, argv);
    fs::path csvPath = root / "scan_output" / "room_scan.csv";

    if (!fs::exists(csvPath))
    {
        std::cerr << "Could not find scan_output/room_scan.csv from root: " << displayPath(root) << std::endl;

        std::cerr << "\nExpected folder structure:\n"
                  << "  <project-root>/\n"
                  << "    scan_output/\n"
                  << "      room_scan.csv\n"
                  << "    tsdf_output/\n"
                  << "      tsdf_room_mesh.ply  optional\n";
        return 1;
    }

    PointCloudData cloud;
    if (!loadRoomScanCsv(csvPath.string(), cloud, gState.maxPoints))
    {
        return 1;
    }

    generateTsdfMeshIfRequested(root);

    fs::path meshPath = findMeshPath(root);
    MeshData mesh;
    bool meshLoaded = false;

    if (!meshPath.empty())
    {
        meshLoaded = loadPlyMesh(meshPath.string(), mesh);
        if (meshLoaded)
            std::cout << "Loaded mesh: " << displayPath(meshPath) << " (" << mesh.vertices.size() << " vertices, " << mesh.indices.size() / 3 << " triangles)" << std::endl;
        else
            std::cout << "Found mesh but could not load: " << displayPath(meshPath) << std::endl;
    }
    else
    {
        std::cout << "No mesh found. M toggle will be disabled until you create one." << std::endl;
    }

    gState.renderDistance = cloud.maxRadius;
    gCamera.reset(cloud.center, cloud.sceneRadius);

    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit())
        return 1;

#if __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#endif

    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow *window = glfwCreateWindow(1600, 1000, "LidarRoomViewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Could not load GLAD." << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 5.0f;
    style.GrabRounding = 5.0f;
    style.WindowBorderSize = 1.0f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);

#if __APPLE__
    ImGui_ImplOpenGL3_Init("#version 410 core");
#else
    ImGui_ImplOpenGL3_Init("#version 330 core");
#endif

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Shader pointShader(Shaders::PointVertex, Shaders::PointFragment);
    Shader meshShader(Shaders::MeshVertex, Shaders::MeshFragment);

    GpuBuffer pointGpu = uploadPointCloud(cloud);
    GpuBuffer meshGpu;
    if (meshLoaded)
        meshGpu = uploadMesh(mesh);

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        glfwPollEvents();
        processInput(window, dt, cloud.center, cloud.sceneRadius);

        if (gState.play)
        {
            gState.buildDegrees += gState.animationSpeedDegPerSec * dt;
            if (gState.buildDegrees >= 360.0f)
            {
                gState.buildDegrees = 360.0f;
                gState.play = false;
            }
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;

        glm::mat4 view = gCamera.viewMatrix();
        glm::mat4 proj = glm::perspective(glm::radians(60.0f), aspect, 0.01f, cloud.sceneRadius * 20.0f + 100.0f);

        glm::vec3 bg = glm::clamp(gState.backgroundColor, glm::vec3(0.0f), glm::vec3(1.0f));
        glViewport(0, 0, width, height);
        glClearColor(bg.r, bg.g, bg.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // put mesh under the points
        if (gState.showMesh && meshLoaded && meshGpu.indexCount > 0)
        {
            glDepthMask(GL_TRUE);
            meshShader.use();
            meshShader.setMat4("uView", view);
            meshShader.setMat4("uProj", proj);
            meshShader.setMat4("uModel", glm::mat4(1.0f));
            meshShader.setVec3("uMeshColor", isBackgroundLight() ? gState.meshColorLight : gState.meshColorDark);
            meshShader.setFloat("uMeshAlpha", gState.meshOpacity);
            meshShader.setVec3("uLightDir", glm::normalize(glm::vec3(-0.4f, -0.8f, -0.25f)));

            glBindVertexArray(meshGpu.vao);
            glDrawElements(GL_TRIANGLES, meshGpu.indexCount, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }

        // points
        if (gState.showPoints)
        {
            glDepthMask(GL_TRUE);
            pointShader.use();
            pointShader.setMat4("uView", view);
            pointShader.setMat4("uProj", proj);
            pointShader.setFloat("uPointSize", gState.pointSize);
            pointShader.setFloat("uRenderDistance", gState.renderDistance);
            pointShader.setFloat("uBuildProgress", std::clamp(gState.buildDegrees / 360.0f, 0.0f, 1.0f));
            pointShader.setVec3("uPointColor", gState.pointColor);
            pointShader.setFloat("uAlpha", 1.0f);

            glBindVertexArray(pointGpu.vao);
            glDrawArrays(GL_POINTS, 0, pointGpu.vertexCount);
            glBindVertexArray(0);
        }

        // UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawUi(csvPath, meshLoaded ? meshPath : fs::path{}, cloud);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    pointGpu.destroy();
    meshGpu.destroy();
    pointShader.destroy();
    meshShader.destroy();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
