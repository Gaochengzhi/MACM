def gcaa_remove_from_list(old_list, index):
    new_list = [-1] * (len(old_list) - 1)
    new_list[:index] = old_list[:index]
    new_list[index:] = old_list[index+1:]
    return new_list

