def gcaa_insert_in_list(old_list, value, index):
    new_list = [-1] * len(old_list)
    new_list[:index] = old_list[:index]
    new_list[index] = value
    new_list[index + 1:] = old_list[index:-1]
    return new_list

