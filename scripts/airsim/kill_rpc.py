#!/usr/bin/env python3

import msgpackrpc
import os

if __name__ == '__main__':
    host = '172.20.0.2'#os.environ['WSL_HOST_IP']
    port = 41451
    msgpackrpc.Client(msgpackrpc.Address(host=host, port=port)).close()
