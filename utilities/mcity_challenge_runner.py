import argparse
import socket


def parse_arguments():
    arg_parser = argparse.ArgumentParser(
        prog='MCity Challenge Runner',
        description='''This program will run the challenge for the MCity event
            based on the inputs provided.''',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    arg_parser.add_argument(
        '-m',
        '--mode',
        metavar='M',
        default='1',
        type=str,
        help='''
            Challenge mode.\n
            '1': Initiate a test run from the start\n
            '2': Initiate the pedestrian crossing & left turn challenge\n
            '3': Initiate the left turn challenge'''
    )
    arg_parser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='The ip address of the host server'
    )
    arg_parser.add_argument(
        '-p',
        '--port',
        metavar='P',
        default=2002,
        type=int,
        help='TCP port used to send socket message'
    )

    args = arg_parser.parse_args()
    args.description = arg_parser.description

    return args


def main():
    args = parse_arguments()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.host, args.port))
    print(s.recv(1024))

    data = args.mode.encode('utf-8')
    s.send(data)

    s.close()


if __name__ == "__main__":
    main()
