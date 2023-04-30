# gui.py
import imgui

nx, ny, nz = 3, 3, 3

def create_gui(config):
    # 创建主窗口
    imgui.begin("Scene Editor", True, imgui.WINDOW_ALWAYS_AUTO_RESIZE)

    # 调整任务空间
    imgui.text("Task Space")
    _, config.task_space["width"] = imgui.slider_float("Width", config.task_space["width"], 1, 100)
    _, config.task_space["height"] = imgui.slider_float("Height", config.task_space["height"], 1, 100)
    _, config.task_space["length"] = imgui.slider_float("Length", config.task_space["length"], 1, 100)

    # 调整障碍物尺寸
    imgui.text("Obstacle")
    _, config.obstacle["width"] = imgui.slider_float("Width", config.obstacle["width"], 1, 100)
    _, config.obstacle["height"] = imgui.slider_float("Height", config.obstacle["height"], 1, 100)
    _, config.obstacle["length"] = imgui.slider_float("Length", config.obstacle["length"], 1, 100)

    # 设置目标点位置和姿态
    for i, target_point in enumerate(config.target_points):
        imgui.text(f"Target Point {i + 1}")
        _, target_point["position"] = imgui.input_float3("Position", *target_point["position"])
        _, target_point["orientation"] = imgui.input_float3("Orientation", *target_point["orientation"])

        if imgui.button(f"Remove Target Point {i + 1}"):
            config.remove_target_point(i)

    if imgui.button("Add Target Point"):
        config.add_target_point()

    # 添加障碍物矩阵编辑器
    imgui.text("Obstacle Matrix")
    global nx, ny, nz
    _, nx = imgui.slider_int("Matrix Width", nx, 1, 10)
    _, ny = imgui.slider_int("Matrix Height", ny, 1, 10)
    _, nz = imgui.slider_int("Matrix Length", nz, 1, 10)

    if len(config.obstacle_matrix) != nz or len(config.obstacle_matrix[0]) != ny or len(config.obstacle_matrix[0][0]) != nx:
        config.obstacle_matrix = [[[0 for _ in range(nx)] for _ in range(ny)] for _ in range(nz)]

    for z in range(nz):
        imgui.text(f"Depth {z + 1}")
        for y in range(ny):
            for x in range(nx):
                _, config.obstacle_matrix[z][y][x] = imgui.checkbox(f"{x + y * nx + z * nx * ny}", config.obstacle_matrix[z][y][x])
                imgui.same_line()
            imgui.new_line()

        if z < nz - 1:
            imgui.separator()


    # imgui.begin_table("matrix", nx, imgui.TABLE_SIZABLE)
    # for i in range(nx):
    #     imgui.table_setup_column(f"Column {i}", imgui.TABLE_COLUMN_NONE)
    # imgui.table_headers_row()

    # for y in range(ny):
    #     for x in range(nx):
    #         idx = y * nx + x
    #         imgui.table_next_row()
    #         imgui.table_set_column_index(x)
    #         _, config.obstacle_matrix[idx] = imgui.checkbox(f"{idx}", config.obstacle_matrix[idx])

    # imgui.end_table()

    # imgui.begin_table("matrix", nx, imgui.TABLE_ROW_BACKGROUND | imgui.TABLE_ROW_SIZABLE)
    # for i in range(nx):
    #     imgui.table_setup_column(f"Column {i}", imgui.TABLE_COLUMN_NONE)
    # imgui.table_headers_row()

    # for y in range(ny):
    #     for x in range(nx):
    #         idx = y * nx + x
    #         imgui.table_next_row(imgui.TABLE_ROW_BACKGROUND)
    #         imgui.table_set_column_index(x)
    #         _, config.obstacle_matrix[idx] = imgui.checkbox(f"{idx}", config.obstacle_matrix[idx])

    # imgui.end_table()

    imgui.text("Matrix Depth")
    for z in range(1, nz):
        imgui.same_line()
        imgui.text(" | ")
    imgui.separator()

    # 保存配置按钮
    if imgui.button("Save Configuration"):
        config.save_to_file()

    imgui.end()
