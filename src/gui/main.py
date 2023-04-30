import pygame
import OpenGL.GL as gl  # 新增
import imgui
from imgui.integrations.pygame import PygameRenderer
from gui import create_gui
from config import Config
from scene import Scene

import pygame.locals

def main():
    # 初始化Pygame
    pygame.init()

    # 创建Pygame窗口
    window = pygame.display.set_mode((1280, 720), pygame.OPENGL | pygame.DOUBLEBUF)  # 修改此行
    pygame.display.set_caption("Scene Editor")

    # 初始化ImGui
    imgui.create_context()
    io = imgui.get_io()
    io.display_size = 1280, 720
    imgui.style_colors_dark()

    # 新增：设置OpenGL上下文
    gl.glClearColor(0.4, 0.4, 0.4, 1)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    imgui_renderer = PygameRenderer()

    # 创建Config实例
    config = Config()
    
    mouse_dragging = False
    last_mouse_x, last_mouse_y = 0, 0

    # 主循环
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            imgui_renderer.process_event(event)

            if event.type == pygame.locals.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button pressed
                    mouse_dragging = True
                    last_mouse_x, last_mouse_y = event.pos

            elif event.type == pygame.locals.MOUSEBUTTONUP:
                if event.button == 1:  # Left mouse button released
                    mouse_dragging = False

            elif event.type == pygame.locals.MOUSEMOTION:
                if mouse_dragging:
                    dx, dy = event.rel
                    scene.camera.rotate(-dy * 0.5, dx * 0.5)
                    last_mouse_x, last_mouse_y = event.pos

        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.new_frame()

        # 创建GUI界面
        create_gui(config)
        scene = Scene(config)

        # 渲染GUI和场景
        window.fill((128, 128, 128))
        scene.render()
        imgui.render()
        imgui_renderer.render(imgui.get_draw_data())

        pygame.display.flip()

    # 清理资源并退出程序
    imgui_renderer.shutdown()
    imgui.destroy_context()
    pygame.quit()

if __name__ == "__main__":
    main()
