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
from rclpy.node import Node
from rcl_interfaces.msg import Log
from explicability_msgs.srv import Question
from langchain.docstore.document import Document
from langchain.vectorstores import Chroma
from langchain.chains import RetrievalQA
from langchain.callbacks.manager import CallbackManager
from langchain.memory import ConversationBufferMemory
from langchain.prompts import PromptTemplate

PATH_TO_PERSIST_DB = "/DB"

class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__('explainability_node')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            10)
        self.subscription
        
        self.srv = self.create_service(Question, "question", self.question_server_callback)
        
        """embeddings = funcion_llamaRos()
        
        self.db = Chroma(
            persist_directory=PATH_TO_PERSIST_DB
            embedding_function = embbedings,
        )
        
        self.retriever = db.as_retriever()
        
        prompt, memory = get_prompt_template(promptTemplate_type=promptTemplate_type, history=use_history)
        
        #llm = load_model(...)
        
        if use_history:
            qa = RetrievalQA.from_chain_type(
                llm=llm,
                chain_type="stuff",  # try other chains types as well. refine, map_reduce, map_rerank
                retriever=self.retriever,
                return_source_documents=True,  # verbose=True,
                callbacks=callback_manager,
                chain_type_kwargs={"prompt": prompt, "memory": memory},
            )
        else:
            qa = RetrievalQA.from_chain_type(
                llm=llm,
                chain_type="stuff",  # try other chains types as well. refine, map_reduce, map_rerank
                retriever=retriever,
                return_source_documents=True,  # verbose=True,
                callbacks=callback_manager,
                chain_type_kwargs={
                    "prompt": prompt,
                },
            )

        self.qa = qa"""
        
    def listener_callback(self, log):
        print('Log Received: "/s"' & log.msg)
        
    def question_server_callback(self, request, response):
        response.answer = request.question + "Respuesta"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ExplainabilityNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


