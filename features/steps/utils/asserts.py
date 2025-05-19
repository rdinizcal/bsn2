def count_matching_elements(list1, list2):
    matching_elements = set(list1) & set(list2)
    # Return the count of matching elements
    return len(matching_elements)
