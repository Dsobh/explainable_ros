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
from rcl_interfaces.msg import Log

from explicability_msgs.srv import Question
from task_creation_chain import TaskCreationChain

from langchain.vectorstores import Chroma
from langchain.prompts import PromptTemplate

from llama_ros.langchain import LlamaROS
from llama_ros.langchain import LlamaROSEmbeddings


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__('explainability_node')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            10)
        self.subscription

        self.srv = self.create_service(
            Question, "question", self.question_server_callback)

        # Model and embeddings from llama_ros
        self.local_llm = LlamaROS(node=self, temp=0.0)
        embeddings = LlamaROSEmbeddings(node=self)

        # Chroma Database
        self.db = Chroma(embedding_function=embeddings)

        self.retriever = self.db.as_retriever(search_kwargs={"k": 10})

        qa = RetrievalQA.from_chain_type(
            llm=local_llm,
            chain_type="stuff",  # try other chains types as well. refine, map_reduce, map_rerank
            retriever=self.retriever,
            return_source_documents=True,  # verbose=True,
            callbacks=callback_manager,
            chain_type_kwargs={"prompt": prompt, "memory": memory},
        )

        self.qa = qa

    # rosout subscription callback
    def listener_callback(self, log):
        print('Log Received: "/s"' & log.msg)
        # Add documents form rosout subscription
        self.db.add_documents(log.msg)

    # Server callback
    def question_server_callback(self, request, response):

        retriever = self.retriever.get_relevant_documents(response.question)

        task_creation_chain = TaskCreationChain.from_llm(self.local_llm)

        # Server response
        response.answer = "Respuesta:" + answer
        return response


def main(args=None):
    rclpy.init(args=args)

    node = ExplainabilityNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
