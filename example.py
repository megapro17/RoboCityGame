from libs import robocitygame as rcg

graph, model, com_flag = rcg.init_game(1, 0)
if com_flag:
    # Начало команд программиста
    model.mov_to_front_point()
    model.mov_to_front_point()
    model.mov_to_right_point()
    model.mov_to_right_point()
    model.mov_to_front_point()
    model.mov_to_front_point()
    model.mov_to_right_point()
    model.mov_to_left_point()
    model.mov_to_front_point()
    model.mov_to_right_point()
    model.mov_to_right_point()
    model.mov_to_front_point()
    model.mov_to_front_point()
    model.mov_to_right_point()
    model.mov_to_front_point()
    #Конец команд 
else:
    print("This track cannot be completed! Change base_info!")
rcg.finalize(model=model, graph=graph)