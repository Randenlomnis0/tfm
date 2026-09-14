#!/usr/bin/env python3

import queue
import threading

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from faster_whisper import WhisperModel


class SpeechClient(Node):

    def __init__(self):

        super().__init__("speech_client")

        self.declare_parameter("output_topic", "/instruction")
        self.declare_parameter("language", "en")
        self.declare_parameter("whisper_model", "base")
        self.declare_parameter("sample_rate", 16000)

        self.topic = self.get_parameter("output_topic").value
        self.language = self.get_parameter("language").value
        self.model_name = self.get_parameter("whisper_model").value
        self.sample_rate = self.get_parameter("sample_rate").value

        self.publisher = self.create_publisher(
            String,
            self.topic,
            10,
        )

        self.whisper = WhisperModel(
            self.model_name,
            device="cpu",
            compute_type="int8"
        )

        self.get_logger().info("Speech client ready.")

    ####################################################################
    # RECORD
    ####################################################################

    def record_until_enter(self):

        frames = []

        recording = True

        def callback(indata, frames_count, time, status):

            if recording:
                frames.append(indata.copy())

        stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.float32,
            callback=callback
        )

        stream.start()

        input("Recording... press ENTER to stop.")

        recording = False

        stream.stop()
        stream.close()

        return np.concatenate(frames, axis=0).flatten()

    ####################################################################
    # SPEECH TO TEXT
    ####################################################################

    def speech_to_text(self, audio):

        task = "transcribe"

        if self.language.lower() != "en":

            task = "translate"

        segments, _ = self.whisper.transcribe(
            audio,
            # language=self.language,
            task=task,
        )

        return "".join(
            segment.text
            for segment in segments
        ).strip()

    ####################################################################
    # LOOP
    ####################################################################

    def run(self):

        while rclpy.ok():

            input("Press ENTER to begin.")

            audio = self.record_until_enter()

            text = self.speech_to_text(audio)

            msg = String()
            msg.data = text

            self.publisher.publish(msg)

            self.get_logger().info(f'Published: "{text}"')


def main():

    rclpy.init()

    node = SpeechClient()

    try:

        node.run()

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()