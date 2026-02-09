#pragma once

#include <GLFW/glfw3.h>
#include <vector>
#include <cassert>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <stdexcept>
#include <unordered_set>

#include "utils/vec.h"

namespace sc
{
namespace internal
{
struct KeyComboHash
{
    size_t operator()(const std::vector<int>& combo) const
    {
        size_t h = 0;
        for (int key : combo)
            h ^= std::hash<int>()(key) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
} // namespace internal

class GLFWRenderer
{
    using KeyCombo = std::vector<int>;
    using KeyHandler = std::function<void()>;
    using MouseMoveHandler = std::function<void(double dx, double dy)>;

    static inline int _glfwRefCount = 0;

public:
    GLFWRenderer(int renderResX,
                 int renderResY,
                 const utils::Vec<int, 2>& windowRes,
                 const char* title = "Renderer")
        : _renderWidth(renderResX)
        , _renderHeight(renderResY)
        , _windowWidth(windowRes[0])
        , _windowHeight(windowRes[1])
        , _buffer(renderResX * renderResY * 3, 0)
    {
        if (_glfwRefCount++ == 0)
        {
            if (!glfwInit())
            {
                --_glfwRefCount;
                throw std::runtime_error("GLFW init failed");
            }
        }

        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

        _window = glfwCreateWindow(_windowWidth, _windowHeight, title, nullptr, nullptr);
        if (!_window)
        {
            if (--_glfwRefCount == 0)
                glfwTerminate();
            throw std::runtime_error("GLFW window creation failed");
        }

        glfwMakeContextCurrent(_window);

        glfwSetWindowUserPointer(_window, this);
        glfwSetMouseButtonCallback(_window, &GLFWRenderer::mouseButtonCallback);
        glfwSetKeyCallback(_window, &GLFWRenderer::keyCallback);
        glfwSetCursorPosCallback(_window, &GLFWRenderer::mouseMoveCallback);

        // IMPORTANT: use framebuffer size, not window size (Retina-safe)
        int fbWidth = 0, fbHeight = 0;
        glfwGetFramebufferSize(_window, &fbWidth, &fbHeight);
        glViewport(0, 0, fbWidth, fbHeight);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glClearColor(0.f, 0.f, 0.f, 1.f);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, 1, 0, 1, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_TEXTURE_2D);

        glGenTextures(1, &_textureId);
        glBindTexture(GL_TEXTURE_2D, _textureId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    }

    GLFWRenderer(GLFWRenderer&& other) noexcept
        : _keyHandlers(std::move(other._keyHandlers))
        , _renderWidth(other._renderWidth)
        , _renderHeight(other._renderHeight)
        , _windowWidth(other._windowWidth)
        , _windowHeight(other._windowHeight)
        , _buffer(std::move(other._buffer))
        , _window(std::exchange(other._window, nullptr))
        , _textureId(std::exchange(other._textureId, 0))
        , _mouseMoveHandler(std::move(other._mouseMoveHandler))
        , _lastMouseX(other._lastMouseX)
        , _lastMouseY(other._lastMouseY)
        , _firstMouse(other._firstMouse)
        , _mouseCaptured(other._mouseCaptured)
    {
        ++_glfwRefCount;
        if (_window)
            glfwSetWindowUserPointer(_window, this);
    }

    GLFWRenderer(const GLFWRenderer&) = delete;
    GLFWRenderer& operator=(const GLFWRenderer&) = delete;
    GLFWRenderer& operator=(GLFWRenderer&&) = delete;

    ~GLFWRenderer()
    {
        if (_window)
        {
            glfwMakeContextCurrent(_window);
            if (_textureId)
                glDeleteTextures(1, &_textureId);
            glfwDestroyWindow(_window);
        }

        if (--_glfwRefCount == 0)
            glfwTerminate();
    }

    void makeContextCurrent()
    {
        if (_window)
            glfwMakeContextCurrent(_window);
    }

    void setPixel(std::size_t x,
                  std::size_t y,
                  const utils::Vec<float, 3>& color)
    {
        assert(x < _renderWidth && y < _renderHeight);

        std::size_t idx =
            (_renderHeight - 1 - y) * _renderWidth * 3 + x * 3;

        _buffer[idx + 0] =
            static_cast<unsigned char>(std::clamp(color[0], 0.f, 1.f) * 255.f);
        _buffer[idx + 1] =
            static_cast<unsigned char>(std::clamp(color[1], 0.f, 1.f) * 255.f);
        _buffer[idx + 2] =
            static_cast<unsigned char>(std::clamp(color[2], 0.f, 1.f) * 255.f);
    }

    /// @brief Upload CPU buffer to GL texture and swap. Calls makeContextCurrent() internally.
    void present()
    {
        if (!_window)
            return;

        glfwMakeContextCurrent(_window);

        glClear(GL_COLOR_BUFFER_BIT);

        glBindTexture(GL_TEXTURE_2D, _textureId);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            static_cast<GLsizei>(_renderWidth),
            static_cast<GLsizei>(_renderHeight),
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            _buffer.data());

        glColor3f(1.f, 1.f, 1.f);

        glBegin(GL_QUADS);
            glTexCoord2f(0.f, 0.f); glVertex2f(0.f, 0.f);
            glTexCoord2f(1.f, 0.f); glVertex2f(1.f, 0.f);
            glTexCoord2f(1.f, 1.f); glVertex2f(1.f, 1.f);
            glTexCoord2f(0.f, 1.f); glVertex2f(0.f, 1.f);
        glEnd();

        glfwSwapBuffers(_window);

        handleSinglePressedKeys();
    }

    /// @brief Backward-compat: present + pollEvents (single-window path).
    void show()
    {
        present();
        glfwPollEvents();
    }

    [[nodiscard]] bool shouldClose() const
    {
        return _window ? glfwWindowShouldClose(_window) : true;
    }

    [[nodiscard]] std::size_t getRenderWidth() const
    {
        return _renderWidth;
    }

    [[nodiscard]] std::size_t getRenderHeight() const
    {
        return _renderHeight;
    }

    void onKey(KeyCombo key, KeyHandler handler)
    {
        if (key.size() == 1)
            _keyHandlers[key[0]] = handler;
        else {
            std::sort(key.begin(), key.end());
            _comboHandlers[key] = std::move(handler);
        }
    }

    void onMouseMove(MouseMoveHandler handler)
    {
        _mouseMoveHandler = std::move(handler);
    }

    void clear(const utils::Vec<float, 3>& color = utils::Vec<float, 3>{0.f, 0.f, 0.f})
    {
        const unsigned char r =
            static_cast<unsigned char>(std::clamp(color[0], 0.f, 1.f) * 255.f);
        const unsigned char g =
            static_cast<unsigned char>(std::clamp(color[1], 0.f, 1.f) * 255.f);
        const unsigned char b =
            static_cast<unsigned char>(std::clamp(color[2], 0.f, 1.f) * 255.f);

        for (std::size_t i = 0; i < _buffer.size(); i += 3)
        {
            _buffer[i + 0] = r;
            _buffer[i + 1] = g;
            _buffer[i + 2] = b;
        }
    }

private:
    std::unordered_map<KeyCombo, KeyHandler, internal::KeyComboHash> _comboHandlers;
    std::unordered_map<int, KeyHandler> _keyHandlers;

    std::size_t _renderWidth;
    std::size_t _renderHeight;
    int _windowWidth;
    int _windowHeight;

    std::vector<unsigned char> _buffer;
    GLFWwindow* _window = nullptr;
    unsigned int _textureId = 0;

    std::vector<int> _pressedKeys;

    MouseMoveHandler _mouseMoveHandler;

    double _lastMouseX = 0.0;
    double _lastMouseY = 0.0;
    bool _firstMouse = true;
    bool _mouseCaptured = false;

    void handleSinglePressedKeys() {
        for (const auto key : _pressedKeys) {
            auto itKey = _keyHandlers.find(key);
            if (itKey == _keyHandlers.end()) return;
            itKey->second();
        }
    }

    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (action != GLFW_PRESS && action != GLFW_RELEASE)
            return;

        auto* self =
            static_cast<GLFWRenderer*>(glfwGetWindowUserPointer(window));

        if (!self || !self->_mouseCaptured)
            return;

        if (action == GLFW_RELEASE) {
            self->_pressedKeys.erase(
                std::remove(
                    self->_pressedKeys.begin(),
                    self->_pressedKeys.end(),
                    key
                ),
                self->_pressedKeys.end()
            );
        } if (action == GLFW_PRESS) {
            self->_pressedKeys.push_back(key);
        }

        std::sort(self->_pressedKeys.begin(), self->_pressedKeys.end());

        auto itCombo = self->_comboHandlers.find(self->_pressedKeys);
        if (itCombo == self->_comboHandlers.end()) return;
        itCombo->second();
    }

    static void mouseMoveCallback(GLFWwindow* window,
                              double xpos,
                              double ypos)
    {
        auto* self =
            static_cast<GLFWRenderer*>(glfwGetWindowUserPointer(window));

        if (!self || !self->_mouseCaptured || !self->_mouseMoveHandler)
            return;

        if (self->_firstMouse)
        {
            self->_lastMouseX = xpos;
            self->_lastMouseY = ypos;
            self->_firstMouse = false;
            return;
        }

        double dx = xpos - self->_lastMouseX;
        double dy = ypos - self->_lastMouseY;

        self->_lastMouseX = xpos;
        self->_lastMouseY = ypos;

        self->_mouseMoveHandler(dx, dy);
    }

    static void mouseButtonCallback(GLFWwindow* window,
                                int button,
                                int action,
                                int mods)
    {
        if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS)
            return;

        auto* self =
            static_cast<GLFWRenderer*>(glfwGetWindowUserPointer(window));

        if (!self)
            return;

        self->_mouseCaptured = !self->_mouseCaptured;

        if (self->_mouseCaptured)
        {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            self->_firstMouse = true;
        }
        else
        {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    }
};


} // namespace sc
