
import time
from typing import List

import rclpy
from simple_node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from builtin_interfaces.msg import Time
from explicability_msgs.srv import Question
from explicability_ros.task_creation_chain import TaskCreationChain

from langchain_community.vectorstores import Chroma
from langchain.docstore.document import Document
from llama_ros.langchain import LlamaROS

from langchain_community.embeddings import HuggingFaceBgeEmbeddings


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__("explainability_node")

        self.logs_number = 0
        self.total_time = 0
        self.embedding_number = 0
        self.msg_queue = []
        self.previous_msg = ""

        # Para gestionar una cola de los 10 mensajes mas recientes y evitar demasiados duplicados
        self.recent_msgs = []
        self.recent_msgs_conter = 0

        # Model and embeddings from llama_ros
        local_llm = LlamaROS(node=self, temp=0.0)
        self.question_chain = TaskCreationChain.from_llm(local_llm)

        # HuggingFace Embeddings Model
        model_name = "mixedbread-ai/mxbai-embed-large-v1"
        model_kwargs = {"device": "cpu"}
        encode_kwargs = {"normalize_embeddings": True}

        # Chroma Database
        self.embeddings_model = HuggingFaceBgeEmbeddings(
            model_name=model_name,
            model_kwargs=model_kwargs,
            encode_kwargs=encode_kwargs
        )
        # embeddings = LlamaROSEmbeddings(node=self)
        self.db = Chroma(
            embedding_function=self.embeddings_model)

        self.retriever = self.db.as_retriever(
            search_type="similarity", search_kwargs={"k": 10})

        self.srv = self.create_service(
            Question, "question", self.question_server_callback,
            callback_group=ReentrantCallbackGroup())

        self.subscription = self.create_subscription(
            Log,
            "/rosout",
            self.listener_callback,
            1000,
            callback_group=ReentrantCallbackGroup())

        self.emb_timer = self.create_timer(0.001, self.emb_cb)

    def listener_callback(self, log: Log) -> None:
        self.logs_number += 1
        self.msg_queue.append(log)
        print(f"Log {self.logs_number}: {log.msg}")

    def emb_cb(self) -> None:

        if self.msg_queue:
            log = self.msg_queue.pop(0)
            start = time.time()

            # Eliminar solo el mensaje anterior
            if log.msg != self.previous_msg:
                msg_sec = log.stamp.sec
                msg_nanosec = log.stamp.nanosec
                unix_timestamp = msg_sec + msg_nanosec / 1e9

                self.db.add_texts(
                    texts=[str(unix_timestamp) + " - " + log.msg])
                self.previous_msg = log.msg
                self.embedding_number += 1

                emb_time = time.time() - start
                self.total_time += emb_time
                print(
                    f"Time to create embedding {self.embedding_number}: {emb_time} | Total time: {self.total_time}")

    def order_retrievals(self, docuemnt_list):

        aux_list = []

        for d in docuemnt_list:
            aux_list.append(d.page_content + "\n")

        return sorted(aux_list)

    def question_server_callback(
        self,
        request: Question.Request,
        response: Question.Response
    ) -> Question.Response:

        docuemnts: List[Document] = self.retriever.get_relevant_documents(
            request.question)

        logs = ""
        sortered_list = self.order_retrievals(docuemnts)

        for i in sortered_list:
            logs += i

        # for d in docuemnts:
        #    logs += d.page_content + "\n"

        answer = self.question_chain.run(logs=logs, question=request.question)

        response.answer = answer
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
