from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.schema.language_model import BaseLanguageModel


class TaskCreationChain(LLMChain):
    """Chain generating tasks."""

    @classmethod
    def from_llm(cls, llm: BaseLanguageModel, verbose: bool = True) -> LLMChain:
        """Get the response parser."""
        task_creation_template = (
            "You are an explainability AI for autonomous robots.\n"

            "You have the following robot logs:\n"
            "{logs}"

            "\n\n### Instruction:\n"
            "Answer the question: {question}"

            "\n\n### Response:\n"
        )
        prompt = PromptTemplate(
            template=task_creation_template,
            input_variables=[
                "question",
                "logs",
            ],
        )
        return cls(prompt=prompt, llm=llm, verbose=verbose)
