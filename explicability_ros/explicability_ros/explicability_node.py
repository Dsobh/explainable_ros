
import time
from typing import List

import rclpy
from simple_node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from explicability_msgs.srv import Question
from explicability_ros.task_creation_chain import TaskCreationChain

from langchain.vectorstores import Chroma
from langchain.docstore.document import Document
from llama_ros.langchain import LlamaROS
from llama_ros.langchain import LlamaROSEmbeddings

from langchain.retrievers.self_query.base import SelfQueryRetriever
from langchain.chains.query_constructor.base import AttributeInfo

from langchain.retrievers import ContextualCompressionRetriever
from langchain.retrievers.document_compressors import LLMChainExtractor

metadata_field_info = [
    AttributeInfo(
        name="ID",
        description="The identification number of a waypoint",
        type="integer",
    ),
    AttributeInfo(
        name="coordinates",
        description="A position in the world with",
        type="vector2",
    ),
]

document_content_description = "Logs produced during the execution of a robot task"


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

        # Chroma Database
        embeddings = LlamaROSEmbeddings(node=self)
        self.db = Chroma(
            embedding_function=embeddings)

        self.retriever = self.db.as_retriever(
            search_type="mmr", search_kwargs={"k": 20})

        # embeddings_filter = EmbeddingsFilter(
        #    embeddings=compressor, similarity_threshold=0.76)

        # Multi Query
        # self.retriever_from_llm = MultiQueryRetriever.from_llm(
        #    retriever=self.db.as_retriever(), llm=local_llm)

        # Metadata info
        # self.retriever = SelfQueryRetriever.from_llm(
        #    local_llm, self.db, document_content_description, metadata_field_info, verbose=True)

        # Context Compression
        # compressor = LLMChainExtractor.from_llm(local_llm)
        # self.compression_retriever = ContextualCompressionRetriever(
        #    base_compressor=compressor, base_retriever=self.retriever)

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

            # Cola de recientes y duplicados
            # if self.recent_msgs_conter < 10:
            #    if not log in self.recent_msgs:
            #        self.recent_msgs.insert(self.recent_msgs_conter, log.msg)
            #        self.db.add_texts(texts=[log.msg])
            #        self.recent_msgs_conter += 1
            #        self.embedding_number += 1
            # else:
            #    self.recent_msgs_conter = 0

            # Eliminar solo el mensaje anterior
            if log.msg != self.previous_msg:
                self.db.add_texts(texts=[log.msg])
                self.previous_msg = log.msg
            else:
                print("este no ha entrado:" + log.msg)

            # self.db.add_texts(texts=[log.msg])
            # self.embedding_number += 1

            emb_time = time.time() - start
            self.total_time += emb_time
            print(
                f"Time to create embedding {self.embedding_number}: {emb_time} | Total time: {self.total_time}")

    def question_server_callback(
        self,
        request: Question.Request,
        response: Question.Response
    ) -> Question.Response:

        docuemnts: List[Document] = self.retriever.get_relevant_documents(
            request.question)

        logs = ""
        for d in docuemnts:
            logs += d.page_content + "\n"

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
