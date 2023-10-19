# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from simple_node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from typing import List

from explicability_msgs.srv import Question
from explicability_node_py.task_creation_chain import TaskCreationChain

from langchain.vectorstores import Chroma
from langchain.docstore.document import Document

from llama_ros.langchain import LlamaROS
from llama_ros.langchain import LlamaROSEmbeddings

from datetime import datetime


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__('explainability_node')

        self.logs_number = 0
        self.msg_queue = []

        # Model and embeddings from llama_ros
        local_llm = LlamaROS(node=self, temp=0.0)
        self.question_chain = TaskCreationChain.from_llm(local_llm)
        embeddings = LlamaROSEmbeddings(node=self)

        # Chroma Database
        self.db = Chroma(
            embedding_function=embeddings)

        self.retriever = self.db.as_retriever(search_kwargs={"k": 10})

        self.srv = self.create_service(
            Question, "question", self.question_server_callback,
            callback_group=ReentrantCallbackGroup())

        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            1000,
            callback_group=ReentrantCallbackGroup())

        self.emb_timer = self.create_timer(
            1, self.emb_cb, callback_group=ReentrantCallbackGroup())

    def listener_callback(self, log: Log) -> None:
        self.logs_number += 1
        self.msg_queue.append(log)
        print("Number of logs received= " + str(self.logs_number) +
              "--- Msg received = " + log.msg)

    def emb_cb(self) -> None:

        if self.msg_queue:
            log = self.msg_queue.pop(0)

            # Add documents form rosout subscription
            now = datetime.now()

            current_time = now.strftime("%H:%M:%S")
            print("Time Before add =", current_time)

            self.db.add_texts(texts=[log.msg])

            now = datetime.now()

            current_time = now.strftime("%H:%M:%S")
            print("Time after add =", current_time)

    # Server callback

    def question_server_callback(
        self,
        request: Question.Request,
        response: Question.Response
    ) -> Question.Response:

        self.get_logger().info("Question Service")

        retriever: List[Document] = self.retriever.get_relevant_documents(
            request.question)

        self.get_logger().info("Data Retrieved")

        logs = ""
        for d in retriever:
            logs += d.page_content + "\n"

        answer = self.question_chain.run(logs=logs, question=request.question)

        # Server response
        response.answer = "Number of msg: " + \
            str(len(self.db.get()['documents'])) + "Respuesta:" + answer
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
