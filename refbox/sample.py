from refbox import RefboxClient
import referee_pb2


r = RefboxClient()

r.connect()
msg = r.receive()
print(msg.command)
print(referee_pb2.SSL_Referee.STOP)
