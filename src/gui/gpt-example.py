import json
import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer

def main():
    # 初始化glfw和imgui
    glfw.init()
    imgui.create_context()
    window = glfw.create_window(1280, 720, "Scene Editor", None, None)
    glfw.make_context_current(window)
    # glfw.set_key_callback(window, key_callback)
    imgui_renderer = GlfwRenderer(window)

    # 配置数据
    config = {
        "task_space": {"width": 1, "height": 1, "length": 1},
        "obstacle": {"width": 1, "height": 1, "length": 1},
        "obstacle_matrix": [],
        "target_point": {"position": [0, 0, 0], "orientation": [0, 0, 0]},
    }

    # 主循环
    while not glfw.window_should_close(window):
        glfw.poll_events()
        imgui.new_frame()

        # 创建GUI界面
        imgui.begin("Scene Editor", True, imgui.WINDOW_ALWAYS_AUTO_RESIZE)

        # 调整任务空间
        _, config["task_space"]["width"] = imgui.slider_float("Task Space Width", config["task_space"]["width"], 1, 10)
        _, config["task_space"]["height"] = imgui.slider_float("Task Space Height", config["task_space"]["height"], 1, 10)
        _, config["task_space"]["length"] = imgui.slider_float("Task Space Length", config["task_space"]["length"], 1, 10)

        # 调整障碍物尺寸
        _, config["obstacle"]["width"] = imgui.slider_float("Obstacle Width", config["obstacle"]["width"], 1, 10)
        _, config["obstacle"]["height"] = imgui.slider_float("Obstacle Height", config["obstacle"]["height"], 1, 10)
        _, config["obstacle"]["length"] = imgui.slider_float("Obstacle Length", config["obstacle"]["length"], 1, 10)

        # 设置目标点位置和姿态
        _, config["target_point"]["position"] = imgui.input_float3("Target Position", *config["target_point"]["position"])
        _, config["target_point"]["orientation"] = imgui.input_float3("Target Orientation", *config["target_point"]["orientation"])

        # 保存配置按钮
        if imgui.button("Save Configuration"):
            with open("config.json", "w") as f:
                json.dump(config, f, indent=2)
        # 添加障碍物矩阵编辑器
        imgui.text("Obstacle Matrix")
        nx, ny, nz = 3, 3, 3  # 设置网格大小
        _, nx = imgui.slider_int("Matrix Width", nx, 1, 10)
        _, ny = imgui.slider_int("Matrix Height", ny, 1, 10)
        _, nz = imgui.slider_int("Matrix Length", nz, 1, 10)

        if len(config["obstacle_matrix"]) != nx * ny * nz:
            config["obstacle_matrix"] = [0] * (nx * ny * nz)

        imgui.begin_table("matrix", nx, imgui.TABLE_SIZING_FIXED_FIT)
        for z in range(nz):
            imgui.table_next_row()
            for y in range(ny):
                for x in range(nx):
                    index = x + y * nx + z * nx * ny
                    imgui.table_set_column_index(x)
                    _, config["obstacle_matrix"][index] = imgui.checkbox(f"{index}", config["obstacle_matrix"][index])
        imgui.end_table()

        # 结束GUI界面并渲染
        imgui.end()
        imgui.render()
        imgui_renderer.render(imgui.get_draw_data())

        # 交换缓冲区并更新窗口
        glfw.swap_buffers(window)

    # 清理资源并退出程序
    imgui_renderer.shutdown()
    imgui.destroy_context()
    glfw.terminate()

if __name__ == "__main__":
    main()
