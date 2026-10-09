#ifndef LYN_SHADER_H
#define LYN_SHADER_H

//#define GLM_FORCE_LEFT_HANDED                                                                      //强制使用左手坐标系
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

class lyn_shader
{
public:
    lyn_shader()
    {
    };

    const char* vertexShaderSource = R"glsl(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aColor;
        out vec3 vertexColor;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        uniform vec3 objectColor;
        void main() {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            vertexColor = aColor - objectColor;
        }
    )glsl";                                                                                        // 顶点着色器源码

    const char* fragmentShaderSource = R"glsl(
            #version 330 core
            in vec3 vertexColor;
            out vec4 FragColor;
            void main() {
                FragColor = vec4(vertexColor, 0.8f);
            }
        )glsl";                                                                                    // 片段着色器源码

    int createShader ()
    {
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);                                    // 创建和编译顶点着色器
        glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
        glCompileShader(vertexShader);
        int success;
        char infoLog[512];
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)                                                                               // 检查顶点着色器编译错误
        {
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);                                // 创建和编译片段着色器
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
        glCompileShader(fragmentShader);
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);                                // 检查片段着色器编译错误
        if (!success)
        {
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        shaderProgram = glCreateProgram();                                                         // 创建着色器程序并链接
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);

        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);                                   // 检查链接错误
        if (!success) {
            glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
            std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }
        glDeleteShader(vertexShader);                                                              // 删除着色器
        glDeleteShader(fragmentShader);

        modelLoc = glGetUniformLocation(shaderProgram, "model");
        viewLoc = glGetUniformLocation(shaderProgram, "view");
        projLoc = glGetUniformLocation(shaderProgram, "projection");
        colorLoc = glGetUniformLocation(shaderProgram, "objectColor");
        return 0;
    }

    GLuint shaderProgram;
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1800.0f / 980.0f, 0.1f, 1000000.0f);;
    GLint modelLoc;
    GLint viewLoc;
    GLint projLoc;
    GLint colorLoc;

};
#endif // LYN_SHADER_H
