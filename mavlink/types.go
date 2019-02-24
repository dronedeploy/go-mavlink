package mavlink

import "encoding/json"

// MavMessage represents a MAV message
type MavMessage interface {
	MsgID() uint8
	MsgName() string
	Pack(p *Packet) error
	Unpack(p *Packet) error
	ToJSON() ([]byte, error)
}

// FromJSON takes an array of bytes and generates a message
func FromJSON(data []byte, msg MavMessage) (MavMessage, error) {
	err := json.Unmarshal(data, msg)
	return msg, err
}
