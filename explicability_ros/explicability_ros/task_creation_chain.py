from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.schema.language_model import BaseLanguageModel


class TaskCreationChain(LLMChain):
    """Chain generating tasks."""

    @classmethod
    def from_llm(cls, llm: BaseLanguageModel, verbose: bool = True) -> LLMChain:
        """Get the response parser."""
        task_creation_template = (
            "<|im_start|>system\n"
            "You are an explainability AI for autonomous robots.\n"
            "You are designed to provide explications about the robot logs, providing only factual information.\n"
            "You have to interpret the logs that have been generated during a run.\n"
            "You should not be overly chatty.\n"

            "Relevant logs are below.:\n"
            "{logs}<|im_end|>\n"

            "<|im_start|>user\n"
            "Given the context information and not prior knowledge, answer the query: {question}<|im_end|>\n"
            "<|im_start|>assistant\n"
        )
        prompt = PromptTemplate(
            template=task_creation_template,
            input_variables=[
                "question",
                "logs",
            ],
        )
        return cls(prompt=prompt, llm=llm, verbose=verbose)
