#MIT License
#
#Copyright (c) 2020 Ubisoft
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

# Modified from Ubisoft code

from godot import exposed, export, Spatial
from godot import *
import os
from enum import IntEnum
from typing import Dict, Mapping, Any, Optional, List
import select
import struct
import json
import logging
import socket

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 12800

logger = logging.getLogger(__name__)


class MessageType(IntEnum):
	"""
	Each message has a integer code to identify it.
	A known issue of this strategy is that it is difficult to sync the code of different kind of clients (blender, vrtist)
	according to changes here. This will be adressed in the future by improving the protocol to include the notion
	of client types.
	Documentation to update if you change this: doc/protocol.md
	"""

	JOIN_ROOM = 1
	LEAVE_ROOM = 3
	LIST_ROOMS = 4
	CONTENT = 5  # Server: ask client to send initial room content; Client: notify server content has been sent
	CLEAR_CONTENT = 6  # Server: ask client to clear its own content before room content is sent
	DELETE_ROOM = 7

	# All joined clients for all rooms
	SET_CLIENT_NAME = 11  # Deprecated
	SEND_ERROR = 12

	# All all joined and un joined clients
	LIST_CLIENTS = 14
	SET_CLIENT_CUSTOM_ATTRIBUTES = 15
	SET_ROOM_CUSTOM_ATTRIBUTES = 16
	SET_ROOM_KEEP_OPEN = 17
	CLIENT_ID = 18  # Client: ask the server to send the unique id string for him; Server: send the unique id string of the client

	CLIENT_UPDATE = 19  # Server: Notify that data of a client have changed
	ROOM_UPDATE = 20  # Server: Notify that data of a room have changed
	ROOM_DELETED = 21  # Server: Notify a room was deleted

	CLIENT_DISCONNECTED = 22  # Server: Notify a client has diconnected

	COMMAND = 100
	DELETE = 101
	CAMERA = 102
	LIGHT = 103
	MESHCONNECTION_DEPRECATED = 104
	RENAME = 105
	DUPLICATE = 106
	SEND_TO_TRASH = 107
	RESTORE_FROM_TRASH = 108
	TEXTURE = 109

	ADD_COLLECTION_TO_COLLECTION = 110
	REMOVE_COLLECTION_FROM_COLLECTION = 111
	ADD_OBJECT_TO_COLLECTION = 112
	REMOVE_OBJECT_FROM_COLLECTION = 113

	ADD_OBJECT_TO_SCENE = 114
	ADD_COLLECTION_TO_SCENE = 115

	INSTANCE_COLLECTION = 116
	COLLECTION = 117
	COLLECTION_REMOVED = 118
	SET_SCENE = 119
	GREASE_PENCIL_MESH = 120
	GREASE_PENCIL_MATERIAL = 121
	GREASE_PENCIL_CONNECTION = 122
	GREASE_PENCIL_TIME_OFFSET = 123
	FRAME_START_END = 124
	ANIMATION = 125

	REMOVE_OBJECT_FROM_SCENE = 126
	REMOVE_COLLECTION_FROM_SCENE = 127

	SCENE = 128
	SCENE_REMOVED = 129

	ADD_OBJECT_TO_VRTIST = 130
	OBJECT_VISIBILITY = 131

	# Start / End a group of command. Allows to inform clients that they must process multiple commands
	# before giving back control to they users.
	GROUP_BEGIN = 132
	GROUP_END = 133

	SCENE_RENAMED = 134

	ADD_KEYFRAME = 135
	REMOVE_KEYFRAME = 136
	QUERY_CURRENT_FRAME = 137
	QUERY_ANIMATION_DATA = 138

	BLENDER_DATA_UPDATE = 139
	CAMERA_ATTRIBUTES = 140
	LIGHT_ATTRIBUTES = 141

	BLENDER_DATA_REMOVE = 142
	BLENDER_DATA_RENAME = 143

	CLEAR_ANIMATIONS = 144
	CURRENT_CAMERA = 145
	SHOT_MANAGER_MONTAGE_MODE = 146
	SHOT_MANAGER_CONTENT = 147
	SHOT_MANAGER_CURRENT_SHOT = 148
	SHOT_MANAGER_ACTION = 149

	BLENDER_DATA_CREATE = 150

	OPTIMIZED_COMMANDS = 200
	TRANSFORM = 201
	MESH = 202
	MATERIAL = 203
	ASSIGN_MATERIAL = 204
	FRAME = 205
	PLAY = 206
	PAUSE = 207

	END_OPTIMIZED_COMMANDS = 999

	CLIENT_ID_WRAPPER = 1000


class LightType(IntEnum):
	SPOT = 0  # directly mapped from Unity enum
	SUN = 1
	POINT = 2
	AREA = 3


class SensorFitMode(IntEnum):
	AUTO = 0
	VERTICAL = 1
	HORIZONTAL = 2


class ClientAttributes:
	"""
	Attributes associated with a client by the server.
	First part is defined by the server, second part is generic and sent by clients to be forwarded to others.
	Clients are free to define custom attributes they need, but some standard names are provided here to ease sync
	between clients of different kind.
	Documentation to update if you change this: doc/protocol.md
	"""

	ID = "id"  # Sent by server only, type = str, the id of the client which is unique for each connected client
	IP = "ip"  # Sent by server only, type = str
	PORT = "port"  # Sent by server only, type = int
	ROOM = "room"  # Sent by server only, type = str

	# Client to server attributes, not used by the server but clients are encouraged to use these keys for the same semantic
	USERNAME = "user_name"  # type = str
	USERCOLOR = "user_color"  # type = float3 (as list)
	USERSCENES = "user_scenes"  # type = dict(str, dict) key = Scene name_full, value = a dictionnary for scene attributes relative to the user
	USERSCENES_FRAME = "frame"  # type = int, can be a field in a user_scenes dict
	USERSCENES_SELECTED_OBJECTS = "selected_objects"  # type = list[string], can be a field in a user_scenes dict
	USERSCENES_VIEWS = (
		"views"  # type dict(str, dict), can be a field in a user_scenes dict; keys are unique ids for the views
	)
	USERSCENES_VIEWS_EYE = "eye"  # type = float3 (as list)
	USERSCENES_VIEWS_TARGET = "target"  # type = float3 (as list)
	USERSCENES_VIEWS_SCREEN_CORNERS = (
		"screen_corners"  # type = list[float3], 4 elements, bottom_left, bottom_right, top_right, top_left
	)


class RoomAttributes:
	"""
	Attributes associated with a room by the server.
	First part is defined by the server, second part is generic and sent by clients to be forwarded to others.
	Clients are free to define custom attributes they need, but some standard names are provided here to ease sync
	between clients of different kind.
	Documentation to update if you change this: doc/protocol.md
	"""

	NAME = "name"  # Sent by server only, type = str, the name of the room which is unique for each room
	KEEP_OPEN = (
		"keep_open"  # Sent by server only, type = bool, indicate if the room should be kept open after all clients left
	)
	COMMAND_COUNT = "command_count"  # Sent by server only, type = bool, indicate how many commands the room contains
	BYTE_SIZE = "byte_size"  # Sent by server only, type = int, indicate the size in byte of the room
	JOINABLE = "joinable"  # Sent by server only, type = bool, indicate if the room is joinable


class ClientDisconnectedException(Exception):
	"""When a client is disconnected and we try to read from it."""


def int_to_bytes(value, size=8):
	return value.to_bytes(size, byteorder="little")


def bytes_to_int(value):
	return int.from_bytes(value, "little")


def int_to_message_type(value):
	return MessageType(value)


def encode_bool(value):
	if value:
		return int_to_bytes(1, 4)
	else:
		return int_to_bytes(0, 4)


def decode_bool(data, index):
	value = bytes_to_int(data[index : index + 4])
	if value == 1:
		return True, index + 4
	else:
		return False, index + 4


def encode_string(value):
	encoded_value = value.encode()
	return int_to_bytes(len(encoded_value), 4) + encoded_value


def decode_string(data, index):
	string_length = bytes_to_int(data[index : index + 4])
	start = index + 4
	end = start + string_length
	value = data[start:end].decode()
	return value, end


def encode_json(value: dict):
	return encode_string(json.dumps(value))


def decode_json(data, index):
	value, end = decode_string(data, index)
	return json.loads(value), end


def encode_float(value):
	return struct.pack("f", value)


def decode_float(data, index):
	return struct.unpack("f", data[index : index + 4])[0], index + 4


def encode_int(value):
	return struct.pack("i", value)


def decode_int(data, index):
	return struct.unpack("i", data[index : index + 4])[0], index + 4


def encode_vector2(value):
	return struct.pack("2f", *(value.x, value.y))


def decode_vector2(data, index):
	return struct.unpack("2f", data[index : index + 2 * 4]), index + 2 * 4


def encode_vector3(value):
	return struct.pack("3f", *(value.x, value.y, value.z))


def decode_vector3(data, index):
	return struct.unpack("3f", data[index : index + 3 * 4]), index + 3 * 4


def encode_vector4(value):
	return struct.pack("4f", *(value[0], value[1], value[2], value[3]))


def decode_vector4(data, index):
	return struct.unpack("4f", data[index : index + 4 * 4]), index + 4 * 4


def encode_matrix(value):
	return (
		encode_vector4(value.col[0])
		+ encode_vector4(value.col[1])
		+ encode_vector4(value.col[2])
		+ encode_vector4(value.col[3])
	)


def decode_matrix(data, index):
	c0, index = decode_vector4(data, index)
	c1, index = decode_vector4(data, index)
	c2, index = decode_vector4(data, index)
	c3, index = decode_vector4(data, index)
	return (c0, c1, c2, c3), index


def encode_color(value):
	if len(value) == 3:
		return struct.pack("4f", *(value[0], value[1], value[2], 1.0))
	else:
		return struct.pack("4f", *(value[0], value[1], value[2], value[3]))


def decode_color(data, index):
	return struct.unpack("4f", data[index : index + 4 * 4]), index + 4 * 4


def encode_quaternion(value):
	return struct.pack("4f", *(value.w, value.x, value.y, value.z))


def decode_quaternion(data, index):
	return struct.unpack("4f", data[index : index + 4 * 4]), index + 4 * 4


def encode_string_array(values):
	buffer = encode_int(len(values))
	for item in values:
		buffer += encode_string(item)
	return buffer


def decode_string_array(data, index):
	count = bytes_to_int(data[index : index + 4])
	index = index + 4
	values = []
	for _ in range(count):
		string, index = decode_string(data, index)
		values.append(string)
	return values, index


def decode_array(data, index, schema, inc):
	count = bytes_to_int(data[index : index + 4])
	start = index + 4
	end = start
	values = []
	for _ in range(count):
		end = start + inc
		values.append(struct.unpack(schema, data[start:end]))
		start = end
	return values, end


def decode_float_array(data, index):
	return decode_array(data, index, "f", 4)


def decode_int_array(data, index):
	count = bytes_to_int(data[index : index + 4])
	start = index + 4
	values = []
	for _ in range(count):
		end = start + 4
		values.extend(struct.unpack("I", data[start:end]))
		start = end
	return values, end


def decode_int2_array(data, index):
	return decode_array(data, index, "2I", 2 * 4)


def decode_int3_array(data, index):
	return decode_array(data, index, "3I", 3 * 4)


def decode_vector3_array(data, index):
	return decode_array(data, index, "3f", 3 * 4)


def decode_vector2_array(data, index):
	return decode_array(data, index, "2f", 2 * 4)


class Command:
	_id = 100

	def __init__(self, command_type: MessageType, data=b"", command_id=0):
		self.data = data or b""
		self.type = command_type
		self.id = command_id
		if command_id == 0:
			self.id = Command._id
			Command._id += 1

	def byte_size(self):
		return 8 + 4 + 2 + len(self.data)

	def to_byte_buffer(self):
		size = int_to_bytes(len(self.data), 8)
		command_id = int_to_bytes(self.id, 4)
		mtype = int_to_bytes(self.type.value, 2)

		return size + command_id + mtype + self.data


class CommandFormatter:
	def format_clients(self, clients):
		s = ""
		for c in clients:
			s += f'   - {c[ClientAttributes.IP]}:{c[ClientAttributes.PORT]} name = "{c[ClientAttributes.USERNAME]}" room = "{c[ClientAttributes.ROOM]}"\n'
		return s

	def format(self, command: Command):

		s = f"={command.type.name}: "

		if command.type == MessageType.LIST_ROOMS:
			rooms, _ = decode_string_array(command.data, 0)
			s += "LIST_ROOMS: "
			if len(rooms) == 0:
				s += "  No rooms"
			else:
				s += f" {len(rooms)} room(s) : {rooms}"
		elif command.type == MessageType.LIST_CLIENTS:
			clients, _ = decode_json(command.data, 0)
			if len(clients) == 0:
				s += "  No clients\n"
			else:
				s += f"  {len(clients)} client(s):\n"
				s += self.format_clients(clients)
		elif command.type == MessageType.SEND_ERROR:
			s += f"ERROR: {decode_string(command.data, 0)[0]}\n"
		else:
			pass

		return s


def recv(socket: socket, size: int):
	"""
	Try to read size bytes from the socket.
	Raise ClientDisconnectedException if the socket is disconnected.
	"""
	result = b""
	while size != 0:
		r, _, _ = select.select([socket], [], [], 0.1)
		if len(r) > 0:
			try:
				tmp = socket.recv(size)
			except (ConnectionAbortedError, ConnectionResetError) as e:
				logger.warning(e)
				raise ClientDisconnectedException()

			if len(tmp) == 0:
				raise ClientDisconnectedException()

			result += tmp
			size -= len(tmp)
	return result


def read_message(socket: socket, timeout: Optional[float] = None) -> Optional[Command]:
	"""
	Try to read a full message from the socket.
	Raise ClientDisconnectedException if the socket is disconnected.
	Return None if no message is waiting on the socket.
	"""
	if not socket:
		logger.warning("read_message called with no socket")
		return None

	select_timeout = timeout if timeout is not None else 0.0001
	r, _, _ = select.select([socket._socket], [], [], select_timeout)
	if len(r) == 0:
		return None

	try:
		prefix_size = 14
		msg = recv(socket, prefix_size)

		frame_size = bytes_to_int(msg[:8])
		command_id = bytes_to_int(msg[8:12])
		message_type = bytes_to_int(msg[12:])

		msg = recv(socket, frame_size)

		return Command(int_to_message_type(message_type), msg, command_id)

	except ClientDisconnectedException:
		raise
	except Exception as e:
		logger.error(e, exc_info=True)
		raise


def read_all_messages(socket: socket, timeout: Optional[float] = None) -> List[Command]:
	"""
	Try to read all messages waiting on the socket.
	Raise ClientDisconnectedException if the socket is disconnected.
	Return empty list if no message is waiting on the socket.
	"""
	received_commands: List[Command] = []
	while True:
		command = read_message(socket, timeout=timeout)
		if command is None:
			break
		received_commands.append(command)
	return received_commands


def write_message(sock: Optional[socket.socket], command: Command):
	if not sock:
		logger.warning("write_message called with no socket")
		return

	buffer = command.to_byte_buffer()

	try:
		_, w, _ = select.select([], [sock], [])
		if sock.sendall(buffer) is not None:
			raise ClientDisconnectedException()
	except (ConnectionAbortedError, ConnectionResetError) as e:
		logger.warning(e)
		raise ClientDisconnectedException()


def make_set_room_attributes_command(room_name: str, attributes: dict):
	return Command(MessageType.SET_ROOM_CUSTOM_ATTRIBUTES, encode_string(room_name) + encode_json(attributes))


def update_attributes_and_get_diff(current: Dict[str, Any], updates: Mapping[str, Any]) -> Dict[str, Any]:
	diff = {}
	for key, value in updates.items():
		if key not in current or current[key] != value:
			current[key] = value
			diff[key] = value
	return diff


def update_named_attributes_and_get_diff(
	current: Dict[str, Dict[str, Any]], updates: Mapping[str, Dict[str, Any]]
) -> Dict[str, Dict[str, Any]]:
	diff = {}
	for name, attrs_updates in updates.items():
		if name not in current:
			current[name] = attrs_updates
			diff[name] = attrs_updates
		else:
			diff[name] = update_attributes_and_get_diff(current[name], attrs_updates)
	return diff


def update_named_attributes(current: Dict[str, Dict[str, Any]], updates: Mapping[str, Dict[str, Any]]):
	for name, attrs_updates in updates.items():
		if name not in current:
			current[name] = attrs_updates
		else:
			attrs = current[name]
			for attr_name, attr_value in attrs_updates.items():
				attrs[attr_name] = attr_value

import logging
import time
from typing import Dict, Any, Mapping, Optional, List, Callable

logger = logging.getLogger() if __name__ == "__main__" else logging.getLogger(__name__)


class Client:
	"""
	The client class is responsible for:
	- handling the connection with the server
	- receiving packet of bytes and convert them to commands
	- send commands
	- maintain an updated view of clients and room states from server's inputs
	"""

	def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
		self.host = host
		self.port = port
		self.pending_commands: List[Command] = []
		self.socket: socket = None

		self.client_id: Optional[str] = None  # Will be filled with a unique string identifying this client
		self.current_custom_attributes: Dict[str, Any] = {}
		self.clients_attributes: Dict[str, Dict[str, Any]] = {}
		self.rooms_attributes: Dict[str, Dict[str, Any]] = {}
		self.current_room: Optional[str] = None

	def __del__(self):
		if self.socket is not None:
			self.disconnect()

	def __enter__(self):
		self.connect()
		return self

	def __exit__(self, *args):
		if self.is_connected():
			self.disconnect()

	def connect(self):
		if self.is_connected():
			raise RuntimeError("Client.connect : already connected")

		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.socket = socket.socket()
			self.socket.connect((self.host, self.port))
			local_address = self.socket.getsockname()
			logger.info(
				"Connecting from local %s:%s to %s:%s",
				local_address[0],
				local_address[1],
				self.host,
				self.port,
			)
			self.send_command(Command(MessageType.CLIENT_ID))
			self.send_command(Command(MessageType.LIST_CLIENTS))
			self.send_command(Command(MessageType.LIST_ROOMS))
		except ConnectionRefusedError:
			self.socket = None
		except ClientDisconnectedException:
			self.handle_connection_lost()
		except Exception as e:
			logger.error("Connection error %s", e, exc_info=True)
			self.socket = None
			raise

	def disconnect(self):
		if self.socket:
			self.socket.shutdown(socket.SHUT_RDWR)
			self.socket.close()
			self.socket = None

	def is_connected(self):
		return self.socket is not None

	def add_command(self, command: Command):
		self.pending_commands.append(command)

	def handle_connection_lost(self):
		logger.info("Connection lost for %s:%s", self.host, self.port)
		# Set socket to None before putting CONNECTION_LIST message to avoid sending/reading new messages
		self.socket = None

	def wait(self, message_type: MessageType) -> bool:
		"""
		Wait for a command of a given message type, the remaining commands are ignored.
		Usually message_type is LEAVING_ROOM.
		"""
		while self.is_connected():
			try:
				received_commands = self.fetch_incoming_commands()
			except ClientDisconnectedException:
				self.handle_connection_lost()
				break
			for command in received_commands:
				if command.type == message_type:
					return True
		return False

	def send_command(self, command: Command):
		try:
			write_message(self.socket, command)
			return True
		except ClientDisconnectedException:
			self.handle_connection_lost()
			return False

	def join_room(self, room_name: str):
		return self.send_command(Command(MessageType.JOIN_ROOM, room_name.encode("utf8"), 0))

	def leave_room(self, room_name: str):
		self.current_room = None
		return self.send_command(Command(MessageType.LEAVE_ROOM, room_name.encode("utf8"), 0))

	def delete_room(self, room_name: str):
		return self.send_command(Command(MessageType.DELETE_ROOM, room_name.encode("utf8"), 0))

	def set_client_attributes(self, attributes: dict):
		diff = update_attributes_and_get_diff(self.current_custom_attributes, attributes)
		if diff == {}:
			return True

		return self.send_command(
			Command(MessageType.SET_CLIENT_CUSTOM_ATTRIBUTES, encode_json(diff), 0)
		)

	def set_room_attributes(self, room_name: str, attributes: dict):
		return self.send_command(make_set_room_attributes_command(room_name, attributes))

	def send_list_rooms(self):
		return self.send_command(Command(MessageType.LIST_ROOMS))

	def set_room_keep_open(self, room_name: str, value: bool):
		return self.send_command(
			Command(
				MessageType.SET_ROOM_KEEP_OPEN, encode_string(room_name) + encode_bool(value), 0
			)
		)

	def _handle_list_client(self, command: Command):
		clients_attributes, _ = decode_json(command.data, 0)
		update_named_attributes(self.clients_attributes, clients_attributes)

	def _handle_list_rooms(self, command: Command):
		rooms_attributes, _ = decode_json(command.data, 0)
		update_named_attributes(self.rooms_attributes, rooms_attributes)

	def _handle_client_id(self, command: Command):
		self.client_id = command.data.decode()

	def _handle_room_update(self, command: Command):
		rooms_attributes_update, _ = decode_json(command.data, 0)
		update_named_attributes(self.rooms_attributes, rooms_attributes_update)

	def _handle_room_deleted(self, command: Command):
		room_name, _ = decode_string(command.data, 0)

		if room_name not in self.rooms_attributes:
			logger.warning("Room %s deleted but no attributes in internal view.", room_name)
			return
		del self.rooms_attributes[room_name]

	def _handle_client_update(self, command: Command):
		clients_attributes_update, _ = decode_json(command.data, 0)
		update_named_attributes(self.clients_attributes, clients_attributes_update)

	def _handle_client_disconnected(self, command: Command):
		client_id, _ = decode_string(command.data, 0)

		if client_id not in self.clients_attributes:
			logger.warning("Client %s disconnected but no attributes in internal view.", client_id)
			return
		del self.clients_attributes[client_id]

	def _handle_join_room(self, command: Command):
		room_name, _ = decode_string(command.data, 0)

		logger.info("Join room %s confirmed by server", room_name)
		self.current_room = room_name

	def _handle_send_error(self, command: Command):
		error_message, _ = decode_string(command.data, 0)

		logger.error("Received error message : %s", error_message)

	_default_command_handlers: Mapping[MessageType, Callable[[Command], None]] = {
		MessageType.LIST_CLIENTS: _handle_list_client,
		MessageType.LIST_ROOMS: _handle_list_rooms,
		MessageType.CLIENT_ID: _handle_client_id,
		MessageType.ROOM_UPDATE: _handle_room_update,
		MessageType.ROOM_DELETED: _handle_room_deleted,
		MessageType.CLIENT_UPDATE: _handle_client_update,
		MessageType.CLIENT_DISCONNECTED: _handle_client_disconnected,
		MessageType.JOIN_ROOM: _handle_join_room,
		MessageType.SEND_ERROR: _handle_send_error,
	}

	def has_default_handler(self, message_type: MessageType):
		return message_type in self._default_command_handlers

	def fetch_incoming_commands(self) -> List[Command]:
		"""
		Gather incoming commands from the socket and return them as a list.
		Process those that have a default handler with the one registered.
		"""
		try:
			received_commands = read_all_messages(self.socket)
		except ClientDisconnectedException:
			self.handle_connection_lost()
			raise

		count = len(received_commands)
		if count > 0:
			logger.debug("Received %d commands", len(received_commands))
		for command in received_commands:
			logger.debug("Received %s", command.type)
			if command.type in self._default_command_handlers:
				self._default_command_handlers[command.type](self, command)

		return received_commands

	def fetch_outgoing_commands(self, commands_send_interval=0):
		"""
		Send commands in pending_commands queue to the server.
		"""
		for idx, command in enumerate(self.pending_commands):
			logger.debug("Send %s (%d / %d)", command.type, idx + 1, len(self.pending_commands))

			if not self.send_command(command):
				break

			if commands_send_interval > 0:
				time.sleep(commands_send_interval)

		self.pending_commands = []

	def fetch_commands(self, commands_send_interval=0) -> List[Command]:
		self.fetch_outgoing_commands(commands_send_interval)
		return self.fetch_incoming_commands()

client1 = Client()
	
@exposed
class new_scene(Spatial):

	# member variables here, example:
	node = export(TextEdit)
	string = export(str, default='foo')

	def _ready(self):
		"""
		Called every time the node is added to the scene.
		Initialization here.
		"""
		node = self.get_node("TextEdit")
		client1.connect()
		string = str(client1.is_connected())
		node.set_text(string) 
		client1.set_client_attributes({ClientAttributes.USERNAME: "c0_name"})
		client1.join_room("example_room")
