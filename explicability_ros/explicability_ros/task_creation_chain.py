from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.schema.language_model import BaseLanguageModel


class TaskCreationChain(LLMChain):
    """Chain generating tasks."""

    @classmethod
    def from_llm(cls, llm: BaseLanguageModel, verbose: bool = True) -> LLMChain:
        """Get the response parser."""
        task_creation_template = (
            "<|im_end|>system\n"
            "You are an explainability AI for autonomous robots.\n"

            "You have the following robot logs:\n"
            "{logs}<|im_end|>\n"

            "<|im_end|>user\n"
            "Answer the question: {question}<|im_end|>\n"
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
