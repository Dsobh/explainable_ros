
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from explicability_msgs.srv import Question

from langchain_community.vectorstores import Chroma
from langchain_community.embeddings import HuggingFaceBgeEmbeddings
from langchain.prompts import PromptTemplate
from llama_ros.langchain import LlamaROS
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough


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

        task_creation_template = (
            "You are an explainability AI tool for ROS 2 mobile robots. "
            "Your goal is to detect possible obstacles. "
            "You have to comparing obstacle detected logs with image-to-text logs. "
            "You have to interpret the robot's data.\n\n"

            "Relevant data:\n"
            "{logs}\n\n"

            "USER: "
            "Given the context information and not prior knowledge, answer the query: {question}\n"
            "ASSISTANT:"
        )
        prompt = PromptTemplate(
            template=task_creation_template,
            input_variables=[
                "question",
                "logs",
            ],
        )

        # Model and embeddings from llama_ros
        local_llm = LlamaROS(temp=0.0)

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
            search_type="similarity", search_kwargs={"k": 5})

        # chain
        def format_docs(docuemnts):
            logs = ""
            sortered_list = self.order_retrievals(docuemnts)

            for l in sortered_list:
                logs += l

        self.question_chain = (
            {
                "logs": self.retriever | format_docs,
                "question": RunnablePassthrough()
            }
            | prompt
            | local_llm
            | StrOutputParser()
        )

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

        answer = self.question_chain.invoke(request.question)
        response.answer = answer
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ExplainabilityNode()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
