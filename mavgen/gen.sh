#!/usr/bin/env bash

go run main.go mavgen.go -f ../mavlink-upstream/message_definitions/v1.0/common.xml -o ../mavlink/common.go
go run main.go mavgen.go -f ../mavlink-upstream/message_definitions/v1.0/ardupilotmega.xml -o ../mavlink/ardupilotmega.go