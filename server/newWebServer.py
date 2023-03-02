import time

import switch
import robotLight
import app
import os
import info

import asyncio
import websockets
from logger import log 

import SpiderG
SpiderG.move_init()

async def check_permit(websocket):
    while True:
        recv_str = await websocket.recv()
        cred_dict = recv_str.split(":")
        if cred_dict[0] == "admin" and cred_dict[1] == "123456":
            response_str = "congratulation, you have connect with server\r\nnow, you can do something else"
            await websocket.send(response_str)
            return True
        else:
            response_str = "sorry, the username or password is wrong, please submit again"
            await websocket.send(response_str)

async def recv_msg(websocket):
    while True:
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }
        data = await websocket.recv()
        if isinstance(data,str):
            log.info('isInst')
            data = data.replace("\"","")
            if 'get_info' == data:
                log.info('get_info')
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info()]
            direction_command(data)


def direction_command(command):
    log.info('direction action')
    directions = ['forward', 'backward', 'left', 'right']
    if command in directions: 
        log.info('Walk {}'.format(command))
        SpiderG.walk(command)



async def set_up(websocket, path):
    await check_permit(websocket)
    await recv_msg(websocket)


if __name__ == '__main__':
    #Setup switches
    log.info('Booting up')
    log.info('Setting up switchs')
    switch.switchSetup()
    log.info('Turning of all switches')
    switch.set_all_switch_off()

    HOST = ''
    PORT = 10223
    BUFSIZ = 1024
    ADDR = (HOST, PORT)

    global flask_app
    flask_app = app.webapp()
    flask_app.startthread()

    try:
        RL = robotLight.RobotLight()
        RL.start()
        RL.breath(70,70,255)
    except:
        pass

    try:
        start_server = websockets.serve(set_up,'0.0.0.0', 8888)
        asyncio.get_event_loop().run_until_complete(start_server)

    except:
        pass

    try:
        asyncio.get_event_loop().run_forever()
    except:
        pass
