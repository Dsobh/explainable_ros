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

#A partir de aqui estos imports seguramente se puedan borrar después de integrar llama_ROS
from langchain.embeddings import HuggingFaceInstructEmbeddings
from langchain.llms import HuggingFacePipeline
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from transformers import (
    GenerationConfig,
    pipeline,
)

import torch
from auto_gptq import AutoGPTQForCausalLM
from huggingface_hub import hf_hub_download
from langchain.llms import LlamaCpp

from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    LlamaForCausalLM,
    LlamaTokenizer,
)

MODELS_PATH = "./models"
PATH_TO_PERSIST_DB = "/DB"
MODEL_ID = "TheBloke/orca_mini_3B-GGML"
MODEL_BASENAME = "orca-mini-3b.ggmlv3.q4_0.bin"
CONTEXT_WINDOW_SIZE = 4096
EMBEDDING_MODEL_NAME = "hkunlp/instructor-large"
MAX_NEW_TOKENS = CONTEXT_WINDOW_SIZE 
N_GPU_LAYERS = 100  # Llama-2-70B has 83 layers
N_BATCH = 512

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
        
        
        #Cosas de llms
        callback_manager = CallbackManager([StreamingStdOutCallbackHandler()])
        model, tokenizer = load_quantized_model_gguf_ggml(MODEL_ID, MODEL_BASENAME, "cuda", LOGGING=logging)
        generation_config = GenerationConfig.from_pretrained(MODEL_ID)
        
        pipe = pipeline(
            "text-generation",
            model=model,
            tokenizer=tokenizer,
            max_length=CONTEXT_WINDOW_SIZE,
            temperature=0.2,
            # top_p=0.95,
            repetition_penalty=1.15,
            generation_config=generation_config,
        )
        
        local_llm = HuggingFacePipeline(pipeline=pipe)
        
        embeddings = HuggingFaceInstructEmbeddings(odel_name=EMBEDDING_MODEL_NAME, model_kwargs={"device": "cuda"}) #funcion_llamaRos()
        
        self.db = Chroma(
            persist_directory=PATH_TO_PERSIST_DB,
            embedding_function = embeddings,
        )
        
        self.retriever = self.db.as_retriever()
        
        prompt, memory = get_prompt_template(promptTemplate_type="llama", history=True)
        
        #llm = load_model(...)
        
        
        qa = RetrievalQA.from_chain_type(
            llm=local_llm,
            chain_type="stuff",  # try other chains types as well. refine, map_reduce, map_rerank
            retriever=self.retriever,
            return_source_documents=True,  # verbose=True,
            callbacks=callback_manager,
            chain_type_kwargs={"prompt": prompt, "memory": memory},
        )
        

        self.qa = qa      
        
    def listener_callback(self, log):
        print('Log Received: "/s"' & log.msg)
        
    def question_server_callback(self, request, response):
        
        res = self.qa(response.question)
        
        answer, docs = res["result"], res["source_documents"]

        # Print the result
        print("\n\n> Question:")
        print(response.question)
        print("\n> Answer:")
        print(answer)
        
        response.answer = "Respuesta:" + answer
        return response
    


def get_prompt_template(promptTemplate_type=None, history=False):
    
    # this is specific to Llama-2. 

    system_prompt = """You are a helpful assistant, you will use the provided context to answer user questions.
    Read the given context before answering questions and think step by step. If you can not answer a user question based on 
    the provided context, inform the user. Do not use any other information for answering user"""

    if promptTemplate_type=="llama":
        B_INST, E_INST = "[INST]", "[/INST]"
        B_SYS, E_SYS = "<<SYS>>\n", "\n<</SYS>>\n\n"
        SYSTEM_PROMPT = B_SYS + system_prompt + E_SYS
        if history:
            instruction = """
            Context: {history} \n {context}
            User: {question}"""

            prompt_template =  B_INST + SYSTEM_PROMPT + instruction + E_INST
            prompt = PromptTemplate(input_variables=["history", "context", "question"], template=prompt_template)
        else:
            instruction = """
            Context: {context}
            User: {question}"""

            prompt_template =  B_INST + SYSTEM_PROMPT + instruction + E_INST
            prompt = PromptTemplate(input_variables=["context", "question"], template=prompt_template)

    else:
        # change this based on the model you have selected. 
        if history:
            prompt_template = system_prompt + """
    
            Context: {history} \n {context}
            User: {question}
            Answer:"""
            prompt = PromptTemplate(input_variables=["history", "context", "question"], template=prompt_template)
        else:
            prompt_template = system_prompt + """
            
            Context: {context}
            User: {question}
            Answer:"""
            prompt = PromptTemplate(input_variables=["context", "question"], template=prompt_template)

    memory = ConversationBufferMemory(input_key="question", memory_key="history")

    return prompt, memory,

def load_quantized_model_gguf_ggml(model_id, model_basename, device_type, logging):

    try:
        logging.info("Using Llamacpp for GGUF/GGML quantized models")
        model_path = hf_hub_download(
            repo_id=model_id,
            filename=model_basename,
            resume_download=True,
            cache_dir=MODELS_PATH,
        )
        kwargs = {
            "model_path": model_path,
            "n_ctx": CONTEXT_WINDOW_SIZE,
            "max_tokens": MAX_NEW_TOKENS,
            "n_batch": N_BATCH,  # set this based on your GPU & CPU RAM
        }
        if device_type.lower() == "mps":
            kwargs["n_gpu_layers"] = 1
        if device_type.lower() == "cuda":
            kwargs["n_gpu_layers"] = N_GPU_LAYERS  # set this based on your GPU

        return LlamaCpp(**kwargs)
    except:
        if "ggml" in model_basename:
            logging.INFO("If you were using GGML model, LLAMA-CPP Dropped Support, Use GGUF Instead")
        return None


def main(args=None):
    rclpy.init(args=args)
    
    node = ExplainabilityNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


