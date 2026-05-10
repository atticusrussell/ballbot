#!/usr/bin/env python
# coding: utf-8

from action_group import action_group

if __name__ == '__main__':
    action = action_group()
    action.set_state(True)
    index = 1
    action.start_action(index)
        

    
